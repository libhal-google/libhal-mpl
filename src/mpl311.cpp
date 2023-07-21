#include "libhal-mpl311/mpl311.hpp"

#include "mpl_reg.hpp"

using namespace std::literals;
namespace hal::mpl311 {

mpl311::mpl311(hal::i2c& p_i2c)
    : m_i2c(&p_i2c)
{}

result<mpl311> mpl311::create(hal::i2c& i2c)
{
    mpl311 mpl_311(i2c);
    if (!mpl_311.driver_configure()) {
        return hal::new_error();
    }

    return mpl_311;
}

/*
* @brief Startup configuration of MPL311X device.
    - Perform WHOAMI check
    - Trigger reset and wait for completion
    - Set altimeter mode
    - Set oversampling ratio to 2^128 (OS128)
    - Enable data ready events for pressure/altitude and temperature
*/
hal::status mpl311::driver_configure()
{
    // sanity check
    std::array<hal::byte, 1> whoami_payload { WHOAMI_R };
    std::array<hal::byte, 1> whoami_buffer {};
    HAL_CHECK(hal::write_then_read(*m_i2c, device_address, whoami_payload, whoami_buffer, hal::never_timeout()));

    if (whoami_buffer[0] != 0xC4) {
        return hal::new_error();
    }

    // software reset
    modify_reg_bits(CTRL_REG1, CTRL_REG1_RST);

    poll_flag(CTRL_REG1, CTRL_REG1_RST, false);

    // set oversampling ratio to 2^128 and set altitude mode
    modify_reg_bits(CTRL_REG1, CTRL_REG1_OS128 | CTRL_REG1_ALT );
    sensor_mode = ALTIMETER_M;

    // enable data ready events for pressure/altitude and temperature
    std::array<hal::byte, 2> dr_payload {
        PT_DATA_CFG_R, 
        PT_DATA_CFG_TDEFE |
        PT_DATA_CFG_PDEFE |
        PT_DATA_CFG_DREM
    };
    HAL_CHECK(hal::write(*m_i2c, device_address, dr_payload, hal::never_timeout()));

    return hal::success();
}

/*
* @brief Set the CTRL_REG1_ALT bit in CTRL_REG1 to the value corresponding to 'mode'
* @param mode: The desired operation mode
*/
hal::status mpl311::set_mode(mpl311_mode_t mode)
{
    // Read value of CTRL_REG1
    std::array<hal::byte, 1> ctrl_payload { CTRL_REG1 };
    std::array<hal::byte, 1> ctrl_buffer;
    HAL_CHECK(hal::write_then_read(*m_i2c, device_address, ctrl_payload, ctrl_buffer, hal::never_timeout()));

    // Set mode ctrl reg bit to binary value of 'mode'
    hal::byte mode_set_reg = ctrl_buffer[0] ^= (-mode ^ ctrl_buffer[0]) & CTRL_REG1_ALT;
    std::array<hal::byte, 2> ctrl_write { CTRL_REG1, mode_set_reg };
    HAL_CHECK(hal::write(*m_i2c, device_address, ctrl_write, hal::never_timeout()));

    sensor_mode = mode;

    return hal::success();
}

/*
* @brief Trigger one-shot measurement by setting 
    CTRL_REG1_OST bit in CTRL_REG1.
*/
hal::status mpl311::initiate_one_shot() 
{
    // Wait for one-shot flag to clear
    poll_flag(CTRL_REG1, CTRL_REG1_OST, false);

    // Set ost bit in CTRL_REG1 - initiate one shot measurement
    HAL_CHECK(modify_reg_bits(CTRL_REG1, CTRL_REG1_OST));

    return hal::success();
}

/*
* @brief Set bits in a register without overwriting existing register state
* @param reg_addr: 8 bit register address
* @param bits_to_set: 8 bit value specifying which bits to set in register
*/
hal::status mpl311::modify_reg_bits(hal::byte reg_addr, hal::byte bits_to_set)
{
    // Read old register value
    std::array<hal::byte, 1> reg_payload { reg_addr };
    std::array<hal::byte, 1> reg_buffer {};
    HAL_CHECK(hal::write_then_read(*m_i2c, device_address, reg_payload, reg_buffer, hal::never_timeout()));

    // Set specified bits while maintaining old values
    hal::byte updated_reg = reg_buffer[0] |= bits_to_set;
    std::array<hal::byte, 2> update_payload { reg_addr, updated_reg };
    HAL_CHECK(hal::write(*m_i2c, device_address, update_payload, hal::never_timeout()));

    return hal::success();
}

/*
* @brief Wait for a specified flag bit in a register to be set to the desired state.
* @param reg: 8 bit value specifying the register address
* @param flag: 8 bit value specifying which bit(s) to check
*/
hal::status mpl311::poll_flag(hal::byte reg, hal::byte flag, bool desired_state)
{   
    std::array<hal::byte, 1> status_payload { reg };
    std::array<hal::byte, 1> status_buffer {};
    bool flag_set = true;

    while (flag_set) {
        hal::result<hal::i2c::transaction_t> res = hal::write_then_read(
                                                            *m_i2c, 
                                                            device_address, 
                                                            status_payload, 
                                                            status_buffer, 
                                                            hal::never_timeout());
        // Only update when no errors (catch device not found)
        if (res) {
            flag_set = desired_state ? ((status_buffer[0] & flag) == 0) : ((status_buffer[0] & flag) != 0);
        }
    }

    return hal::success();
}

/*
* @brief Read pressure data from OUT_T_MSB_R and OUT_T_LSB_R
*   and perform temperature conversion to celsius.
*/
hal::result<mpl311::temperature_read_t> mpl311::t_read()
{
    constexpr float temp_conversion_factor = 256.0f;

    initiate_one_shot();

    poll_flag(STATUS_R, STATUS_TDR, true);

    // Read data from OUT_T_MSB_R and OUT_T_LSB_R
    std::array<hal::byte, 1> temp_payload = { OUT_T_MSB_R };
    std::array<hal::byte, 2> temp_buffer {};
    HAL_CHECK(hal::write_then_read(*m_i2c, device_address, temp_payload, temp_buffer, hal::never_timeout()));

    int16_t temp_reading = int16_t(temp_buffer[0]) << 8 | int16_t(temp_buffer[1]);
    return mpl311::temperature_read_t {
        static_cast<float>(temp_reading) / temp_conversion_factor,
    };
}

/*
* @brief Read pressure data from OUT_P_MSB_R, OUT_P_CSB_R, and OUT_P_LSB_R
*   and perform pressure conversion to kilopascals.
*/
hal::result<mpl311::pressure_read_t> mpl311::p_read()
{
    // Note: 64 -> Pa, 6400 -> kPa
    constexpr float pressure_conversion_factor = 64.0f;

    if (sensor_mode != BAROMETER_M) {
        set_mode(BAROMETER_M);
    }

    initiate_one_shot();

    poll_flag(STATUS_R, STATUS_PDR, true);

    // Read data from OUT_P_MSB_R, OUT_P_CSB_R, and OUT_P_LSB_R
    std::array<hal::byte, 1> pres_payload = { OUT_P_MSB_R };
    std::array<hal::byte, 3> pres_buffer {};
    HAL_CHECK(hal::write_then_read(*m_i2c, device_address, pres_payload, pres_buffer, hal::never_timeout()));

    uint32_t pressure_reading = uint32_t(pres_buffer[0]) << 16 
                              | uint32_t(pres_buffer[1]) << 8 
                              | uint32_t(pres_buffer[2]);

    return mpl311::pressure_read_t {
       static_cast<float>(pressure_reading) / pressure_conversion_factor,
    };
}

/*
* @brief Read pressure data from OUT_P_MSB_R, OUT_P_CSB_R, and OUT_P_LSB_R
*   and perform altitude conversion to meters.
*/
hal::result<mpl311::altitude_read_t> mpl311::a_read()
{
    constexpr float altitude_conversion_factor = 65536.0f;

    if (sensor_mode != ALTIMETER_M) {
        set_mode(ALTIMETER_M);
    }

    initiate_one_shot();

    poll_flag(STATUS_R, STATUS_PDR, true);

    // Read data from OUT_P_MSB_R, OUT_P_CSB_R, and OUT_P_LSB_R
    std::array<hal::byte, 1> alt_payload = { OUT_P_MSB_R };
    std::array<hal::byte, 3> alt_buffer {};
    HAL_CHECK(hal::write_then_read(*m_i2c, device_address, alt_payload, alt_buffer, hal::never_timeout()));

    int32_t alt_reading = int32_t(alt_buffer[0]) << 24 
                        | int32_t(alt_buffer[1]) << 16 
                        | int32_t(alt_buffer[2]) << 8;

    return mpl311::altitude_read_t {
        static_cast<float>(alt_reading) / altitude_conversion_factor,
    };
}

/*
* @brief Set sea level pressure (Barometric input for altitude calculations)
         in BAR_IN_MSB_R and BAR_IN_LSB_R registers
* @param sea_level_pressure: Sea level pressure in Pascals. 
*      - Default value on startup is 101,326 Pa.
*/
hal::status mpl311::set_sea_pressure(float sea_level_pressure)
{
    // divide by 2 to convert to 2Pa per LSB
    uint16_t two_pa = (sea_level_pressure / 2);

    // write result to register
    std::array<hal::byte, 3> slp_payload = { 
        BAR_IN_MSB_R, 
        two_pa >> 8,  // msb
        two_pa & 0xFF // lsb
    };

    HAL_CHECK(hal::write(*m_i2c, device_address, slp_payload, hal::never_timeout()));

    return hal::success();
}

/*
* @brief Set altitude offset in OFF_H_R
* @param offset Offset value in meters, from -127 to 128
*/
hal::status mpl311::set_altitude_offset(int8_t offset)
{
    std::array<hal::byte, 2> offset_payload = { OFF_H_R, hal::byte(offset) };
    HAL_CHECK(hal::write(*m_i2c, device_address, offset_payload, hal::never_timeout()));

    return hal::success();
}

}  // namespace hal::mpl311
