#include "libhal-mpl/mpl.hpp"

#include "mpl_reg.hpp"

using namespace std::literals;
namespace hal::mpl {

mpl::mpl(hal::i2c& p_i2c)
    : m_i2c(&p_i2c)
{}

result<mpl> mpl::create(hal::i2c& i2c)
{
    mpl mpl_dev(i2c);
    HAL_CHECK(mpl_dev.init());
    return mpl_dev;
}

hal::status mpl::init()
{
    // sanity check
    auto whoami_buffer = HAL_CHECK(hal::write_then_read<1>(*m_i2c, device_address, std::array<hal::byte, 1>{ whoami_r }, hal::never_timeout()));

    if (whoami_buffer[0] != 0xC4) {
        return hal::new_error();
    }

    // software reset
    modify_reg_bits(ctrl_reg1, ctrl_reg1_rst);

    poll_flag(ctrl_reg1, ctrl_reg1_rst, false);

    // set oversampling ratio to 2^128 and set altitude mode
    modify_reg_bits(ctrl_reg1, ctrl_reg1_os128 | ctrl_reg1_alt );
    sensor_mode = mpl_mode_t::ALTIMETER_M;

    // enable data ready events for pressure/altitude and temperature
    std::array<hal::byte, 2> dr_payload {
        pt_data_cfg_r, 
        pt_data_cfg_tdefe |
        pt_data_cfg_pdefe |
        pt_data_cfg_drem
    };
    HAL_CHECK(hal::write(*m_i2c, device_address, dr_payload, hal::never_timeout()));

    return hal::success();
}

hal::status mpl::set_mode(mpl_mode_t mode)
{
    // Read value of ctrl_reg1
    auto ctrl_buffer = HAL_CHECK(hal::write_then_read<1>(*m_i2c, device_address, std::array<hal::byte, 1>{ ctrl_reg1 }, hal::never_timeout()));
    
    hal::byte reg_val = ctrl_buffer[0];
    if ((reg_val & ctrl_reg1_alt) != static_cast<int>(mode)) {
        // Set mode ctrl reg bit to binary value of 'mode'
        if (mode == mpl_mode_t::BAROMETER_M) {
            reg_val &= ~ctrl_reg1_alt; // Set the bit to 0
        } else {
            reg_val |= ctrl_reg1_alt;  // Set the bit to 1
        }
    }

    HAL_CHECK(hal::write(*m_i2c, device_address, std::array<hal::byte, 2>{ ctrl_reg1, reg_val }, hal::never_timeout()));

    sensor_mode = mode;

    return hal::success();
}

hal::status mpl::initiate_one_shot() 
{
    // Wait for one-shot flag to clear
    poll_flag(ctrl_reg1, ctrl_reg1_ost, false);

    // Set ost bit in ctrl_reg1 - initiate one shot measurement
    HAL_CHECK(modify_reg_bits(ctrl_reg1, ctrl_reg1_ost));

    return hal::success();
}

hal::status mpl::modify_reg_bits(hal::byte reg_addr, hal::byte bits_to_set)
{
    // Read old register value
    auto reg_buffer = HAL_CHECK(hal::write_then_read<1>(*m_i2c, device_address, std::array<hal::byte, 1>{ reg_addr }, hal::never_timeout()));

    // Set specified bits while maintaining old values
    hal::byte updated_reg = reg_buffer[0] | bits_to_set;
    HAL_CHECK(hal::write(*m_i2c, device_address, std::array<hal::byte, 2>{ reg_addr, updated_reg }, hal::never_timeout()));

    return hal::success();
}

hal::status mpl::poll_flag(hal::byte reg, hal::byte flag, bool desired_state)
{   
    std::array<hal::byte, 1> status_payload { reg };
    std::array<hal::byte, 1> status_buffer {};
    bool flag_set = true;

    while (flag_set) {
        if (hal::write_then_read(
                *m_i2c, 
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

hal::result<mpl::temperature_read_t> mpl::read_temperature()
{
    constexpr float temp_conversion_factor = 256.0f;

    initiate_one_shot();

    poll_flag(status_r, status_tdr, true);

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
        set_mode(mpl_mode_t::BAROMETER_M);
    }

    initiate_one_shot();

    poll_flag(status_r, status_pdr, true);

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
        set_mode(mpl_mode_t::ALTIMETER_M);
    }

    initiate_one_shot();

    poll_flag(status_r, status_pdr, true);

    // Read data from out_p_msb_r, out_p_csb_r, and out_p_lsb_r
    auto alt_buffer = HAL_CHECK(hal::write_then_read<3>(*m_i2c, device_address, std::array<hal::byte, 1>{ out_p_msb_r }, hal::never_timeout()));

    int32_t alt_reading = int32_t(alt_buffer[0]) << 24 
                        | int32_t(alt_buffer[1]) << 16 
                        | int32_t(alt_buffer[2]) << 8;

    return mpl::altitude_read_t {
        static_cast<float>(alt_reading) / altitude_conversion_factor,
    };
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

}  // namespace hal::mpl
