#include "libhal-mpl/mpl.hpp"

#include "mpl_reg.hpp"

using namespace std::literals;
namespace hal::mpl {

/***********************************************
 *  Free Functions
 ***********************************************/

/**
* @brief Set the ctrl_reg1_alt bit in ctrl_reg1 to the value corresponding to 'mode'
* @param p_i2c The I2C peripheral used for communication with the device.
* @param mode: The desired operation mode
*/
hal::status set_mode(hal::i2c* p_i2c, mpl::mpl_mode_t mode)
{
    // Read value of ctrl_reg1
    auto ctrl_buffer = HAL_CHECK(hal::write_then_read<1>(*p_i2c, device_address, std::array<hal::byte, 1>{ ctrl_reg1 }, hal::never_timeout()));
    
    hal::byte reg_val = ctrl_buffer[0];
    if ((reg_val & ctrl_reg1_alt) != static_cast<int>(mode)) {
        // Set mode ctrl reg bit to binary value of 'mode'
        if (mode == mpl::mpl_mode_t::BAROMETER_M) {
            reg_val &= ~ctrl_reg1_alt; // Set the bit to 0
        } else {
            reg_val |= ctrl_reg1_alt;  // Set the bit to 1
        }
    }

    HAL_CHECK(hal::write(*p_i2c, device_address, std::array<hal::byte, 2>{ ctrl_reg1, reg_val }, hal::never_timeout()));

    return hal::success();
}

/**
* @brief Set bits in a register without overwriting existing register state
* @param p_i2c The I2C peripheral used for communication with the device.
* @param reg_addr: 8 bit register address
* @param bits_to_set: 8 bit value specifying which bits to set in register
*/
hal::status modify_reg_bits(hal::i2c* p_i2c, hal::byte reg_addr, hal::byte bits_to_set)
{
    // Read old register value
    auto reg_buffer = HAL_CHECK(hal::write_then_read<1>(*p_i2c, device_address, std::array<hal::byte, 1>{ reg_addr }, hal::never_timeout()));

    // Set specified bits while maintaining old values
    hal::byte updated_reg = reg_buffer[0] | bits_to_set;
    HAL_CHECK(hal::write(*p_i2c, device_address, std::array<hal::byte, 2>{ reg_addr, updated_reg }, hal::never_timeout()));

    return hal::success();
}

/**
* @brief Wait for a specified flag bit in a register to be set to the desired state.
* @param p_i2c The I2C peripheral used for communication with the device.
* @param reg: 8 bit value specifying the register address
* @param flag: 8 bit value specifying which bit(s) to check
*/
hal::status poll_flag(hal::i2c* p_i2c, hal::byte reg, hal::byte flag, bool desired_state)
{   
    std::array<hal::byte, 1> status_payload { reg };
    std::array<hal::byte, 1> status_buffer {};
    bool flag_set = true;

    while (flag_set) {
        if (hal::write_then_read(
                *p_i2c, 
                device_address, 
                status_payload, 
                status_buffer, 
                hal::never_timeout())
        ) {
            // Only update when no errors (catch device not found)
            flag_set = desired_state ? ((status_buffer[0] & flag) == 0) : ((status_buffer[0] & flag) != 0);
        }
    }

    return hal::success();
}

/**
* @brief Trigger one-shot measurement by setting ctrl_reg1_ost bit in ctrl_reg1.
* @param p_i2c The I2C peripheral used for communication with the device.
*/
hal::status initiate_one_shot(hal::i2c* p_i2c) 
{
    // Wait for one-shot flag to clear
    poll_flag(p_i2c, ctrl_reg1, ctrl_reg1_ost, false);

    // Set ost bit in ctrl_reg1 - initiate one shot measurement
    HAL_CHECK(modify_reg_bits(p_i2c, ctrl_reg1, ctrl_reg1_ost));

    return hal::success();
}

/***********************************************
 *  MPL Class Functions
 ***********************************************/

mpl::mpl(hal::i2c& p_i2c)
    : m_i2c(&p_i2c)
{}

result<mpl> mpl::create(hal::i2c& p_i2c)
{
    mpl mpl_dev(p_i2c);
    
    // sanity check
    auto whoami_buffer = HAL_CHECK(hal::write_then_read<1>(p_i2c, device_address, std::array<hal::byte, 1>{ whoami_r }, hal::never_timeout()));

    if (whoami_buffer[0] != 0xC4) {
        return hal::new_error();
    }

    // software reset
    modify_reg_bits(&p_i2c, ctrl_reg1, ctrl_reg1_rst);

    poll_flag(&p_i2c, ctrl_reg1, ctrl_reg1_rst, false);

    // set oversampling ratio to 2^128 and set altitude mode
    modify_reg_bits(&p_i2c, ctrl_reg1, ctrl_reg1_os128 | ctrl_reg1_alt );
    mpl_dev.sensor_mode = mpl_mode_t::ALTIMETER_M;

    // enable data ready events for pressure/altitude and temperature
    std::array<hal::byte, 2> dr_payload {
        pt_data_cfg_r, 
        pt_data_cfg_tdefe |
        pt_data_cfg_pdefe |
        pt_data_cfg_drem
    };
    HAL_CHECK(hal::write(p_i2c, device_address, dr_payload, hal::never_timeout()));

    return mpl_dev;
}

hal::status mpl::set_sea_pressure(float sea_level_pressure)
{
    // divide by 2 to convert to 2Pa per LSB
    uint16_t two_pa = (sea_level_pressure / 2);

    // write result to register
    std::array<hal::byte, 3> slp_payload = { 
        bar_in_msb_r, 
        two_pa >> 8,  // msb
        two_pa & 0xFF // lsb
    };

    HAL_CHECK(hal::write(*m_i2c, device_address, slp_payload, hal::never_timeout()));

    return hal::success();
}

hal::status mpl::set_altitude_offset(int8_t offset)
{
    std::array<hal::byte, 2> offset_payload = { off_h_r, hal::byte(offset) };
    HAL_CHECK(hal::write(*m_i2c, device_address, offset_payload, hal::never_timeout()));

    return hal::success();
}

hal::result<mpl::temperature_read_t> mpl::read_temperature()
{
    constexpr float temp_conversion_factor = 256.0f;

    initiate_one_shot(m_i2c);

    poll_flag(m_i2c, status_r, status_tdr, true);

    // Read data from out_t_msb_r and out_t_lsb_r
    auto temp_buffer = HAL_CHECK(hal::write_then_read<2>(*m_i2c, device_address, std::array<hal::byte, 1>{ out_t_msb_r }, hal::never_timeout()));

    int16_t temp_reading = int16_t(temp_buffer[0]) << 8 | int16_t(temp_buffer[1]);
    return mpl::temperature_read_t {
        static_cast<float>(temp_reading) / temp_conversion_factor,
    };
}

hal::result<mpl::pressure_read_t> mpl::read_pressure()
{
    // Note: 64 -> Pa, 6400 -> kPa
    constexpr float pressure_conversion_factor = 64.0f;

    if (sensor_mode != mpl_mode_t::BAROMETER_M) {
        set_mode(m_i2c, mpl_mode_t::BAROMETER_M);
        sensor_mode = mpl_mode_t::BAROMETER_M;
    }

    initiate_one_shot(m_i2c);

    poll_flag(m_i2c, status_r, status_pdr, true);

    // Read data from out_p_msb_r, out_p_csb_r, and out_p_lsb_r
    auto pres_buffer = HAL_CHECK(hal::write_then_read<3>(*m_i2c, device_address, std::array<hal::byte, 1>{ out_p_msb_r }, hal::never_timeout()));

    uint32_t pressure_reading = uint32_t(pres_buffer[0]) << 16 
                              | uint32_t(pres_buffer[1]) << 8 
                              | uint32_t(pres_buffer[2]);

    return mpl::pressure_read_t {
       static_cast<float>(pressure_reading) / pressure_conversion_factor,
    };
}

hal::result<mpl::altitude_read_t> mpl::read_altitude()
{
    constexpr float altitude_conversion_factor = 65536.0f;

    if (sensor_mode != mpl_mode_t::ALTIMETER_M) {
        set_mode(m_i2c, mpl_mode_t::ALTIMETER_M);
        sensor_mode = mpl_mode_t::ALTIMETER_M;
    }

    initiate_one_shot(m_i2c);

    poll_flag(m_i2c, status_r, status_pdr, true);

    // Read data from out_p_msb_r, out_p_csb_r, and out_p_lsb_r
    auto alt_buffer = HAL_CHECK(hal::write_then_read<3>(*m_i2c, device_address, std::array<hal::byte, 1>{ out_p_msb_r }, hal::never_timeout()));

    int32_t alt_reading = int32_t(alt_buffer[0]) << 24 
                        | int32_t(alt_buffer[1]) << 16 
                        | int32_t(alt_buffer[2]) << 8;

    return mpl::altitude_read_t {
        static_cast<float>(alt_reading) / altitude_conversion_factor,
    };
}

}  // namespace hal::mpl
