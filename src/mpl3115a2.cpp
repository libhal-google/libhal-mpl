#include <libhal-mpl/mpl3115a2.hpp>

#include <array>

#include <libhal-util/i2c.hpp>

#include "mpl3115a2_reg.hpp"

using namespace std::literals;
namespace hal::mpl {
namespace {
/**
 * @brief Set the ctrl_reg1_alt bit in ctrl_reg1 to the value corresponding to
 * 'mode'
 * @param p_i2c The I2C peripheral used for communication with the device.
 * @param p_mode: The desired operation mode
 */
hal::status set_mode(hal::i2c* p_i2c, mpl3115a2::mode p_mode)
{
  // Read value of ctrl_reg1
  auto ctrl_buffer =
    HAL_CHECK(hal::write_then_read<1>(*p_i2c,
                                      device_address,
                                      std::array<hal::byte, 1>{ ctrl_reg1 },
                                      hal::never_timeout()));

  hal::byte reg_val = ctrl_buffer[0];
  if ((reg_val & ctrl_reg1_alt) != static_cast<int>(p_mode)) {
    // Set mode ctrl reg bit to binary value of 'mode'
    if (p_mode == mpl3115a2::mode::barometer) {
      reg_val &= ~ctrl_reg1_alt;  // Set the bit to 0
    } else {
      reg_val |= ctrl_reg1_alt;  // Set the bit to 1
    }
  }

  HAL_CHECK(hal::write(*p_i2c,
                       device_address,
                       std::array<hal::byte, 2>{ ctrl_reg1, reg_val },
                       hal::never_timeout()));

  return hal::success();
}

struct modify_reg_param_t
{
  hal::byte address;
  hal::byte bits_to_set;
};

/**
 * @brief Set bits in a register without overwriting existing register state
 * @param p_i2c The I2C peripheral used for communication with the device.
 * @param p_reg_addr: 8 bit register address
 * @param p_bits_to_set: 8 bit value specifying which bits to set in register
 */
hal::status modify_reg_bits(hal::i2c* p_i2c, modify_reg_param_t p_reg)
{
  // Read old register value
  auto reg_buffer =
    HAL_CHECK(hal::write_then_read<1>(*p_i2c,
                                      device_address,
                                      std::array<hal::byte, 1>{ p_reg.address },
                                      hal::never_timeout()));

  // Set specified bits while maintaining old values
  hal::byte updated_reg = reg_buffer[0] | p_reg.bits_to_set;
  HAL_CHECK(hal::write(*p_i2c,
                       device_address,
                       std::array<hal::byte, 2>{ p_reg.address, updated_reg },
                       hal::never_timeout()));

  return hal::success();
}

/**
* @brief Wait for the reset bit in ctrl_reg1 to be set.
         Catches and ignores expected std::errc::no_such_device_or_address.
* @param p_i2c The I2C peripheral used for communication with the device.
*/
hal::status poll_reset(hal::i2c* p_i2c)
{
  bool flag_set = true;
  uint16_t retries = 0;

  // Lambda function to poll ctrl_reg1 reset flag
  auto poll_function = [&p_i2c, &flag_set]() -> hal::status {
    std::array<hal::byte, 1> status_buffer{};
    HAL_CHECK(hal::write_then_read(*p_i2c,
                                   device_address,
                                   std::array<hal::byte, 1>{ ctrl_reg1 },
                                   status_buffer,
                                   hal::never_timeout()));
    flag_set = ((status_buffer[0] & ctrl_reg1_rst) != 0);
    return hal::success();
  };

  // std::errc error code handler. std::errc::no_such_device_or_address is
  // expected during reset as the device comes online.
  auto err_handler = [](std::errc e_code) -> hal::status {
    if (e_code != std::errc::no_such_device_or_address) {
      return hal::new_error(e_code);
    }
    return hal::success();
  };

  // Perform polling
  while (flag_set && (retries < mpl3115a2::default_max_polling_retries)) {
    HAL_CHECK(hal::attempt(poll_function, err_handler));
    retries++;
  }

  return hal::success();
}

struct poll_flag_param_t
{

  /// 8 bit value specifying the register address
  hal::byte address;
  /// 8 bit value specifying which bit(s) to check
  hal::byte flag;
  /// The state of the bit to finish polling
  bool desired_state;
};

/**
 * @brief Wait for a specified flag bit in a register to be set to the desired
 * state.
 * @param p_i2c The I2C peripheral used for communication with the device.
 * reached, the function will exit.
 */
hal::status poll_flag(hal::i2c* p_i2c, poll_flag_param_t p_poll)
{
  std::array<hal::byte, 1> status_payload{ p_poll.address };
  std::array<hal::byte, 1> status_buffer{};
  uint16_t retries = 0;
  bool flag_set = true;

  while (flag_set && (retries < mpl3115a2::default_max_polling_retries)) {
    HAL_CHECK(hal::write_then_read(*p_i2c,
                                   device_address,
                                   status_payload,
                                   status_buffer,
                                   hal::never_timeout()));

    if (p_poll.desired_state) {
      flag_set = ((status_buffer[0] & p_poll.flag) == 0);
    } else {
      flag_set = ((status_buffer[0] & p_poll.flag) != 0);
    }
    retries++;
  }

  return hal::success();
}

/**
 * @brief Trigger one-shot measurement by setting ctrl_reg1_ost bit in
 * ctrl_reg1.
 * @param p_i2c The I2C peripheral used for communication with the device.
 */
hal::status initiate_one_shot(hal::i2c* p_i2c)
{
  // Wait for one-shot flag to clear
  poll_flag(
    p_i2c,
    { .address = ctrl_reg1, .flag = ctrl_reg1_ost, .desired_state = false });

  // Set ost bit in ctrl_reg1 - initiate one shot measurement
  HAL_CHECK(modify_reg_bits(
    p_i2c, { .address = ctrl_reg1, .bits_to_set = ctrl_reg1_ost }));

  return hal::success();
}
}  // namespace

mpl3115a2::mpl3115a2(hal::i2c& p_i2c)
  : m_i2c(&p_i2c)
  , m_sensor_mode(mode::altimeter)
{
}

result<mpl3115a2> mpl3115a2::create(hal::i2c& p_i2c, mpl_os_rate ctrl_reg1_os)
{
  mpl3115a2 mpl_dev(p_i2c);

  // sanity check
  auto whoami_buffer =
    HAL_CHECK(hal::write_then_read<1>(p_i2c,
                                      device_address,
                                      std::array<hal::byte, 1>{ whoami_r },
                                      hal::never_timeout()));

  if (whoami_buffer[0] != 0xC4) {
    return hal::new_error(std::errc::no_such_device);
  }

  // software reset
  modify_reg_bits(&p_i2c,
                  { .address = ctrl_reg1, .bits_to_set = ctrl_reg1_rst });

  poll_reset(&p_i2c);

  // set oversampling ratio and set altitude mode
  modify_reg_bits(&p_i2c, ctrl_reg1, ctrl_reg1_os | ctrl_reg1_alt);
  mpl_dev.m_sensor_mode = mode::altimeter;

  // enable data ready events for pressure/altitude and temperature
  std::array<hal::byte, 2> dr_payload{
    pt_data_cfg_r, pt_data_cfg_tdefe | pt_data_cfg_pdefe | pt_data_cfg_drem
  };
  HAL_CHECK(
    hal::write(p_i2c, device_address, dr_payload, hal::never_timeout()));

  return mpl_dev;
}

hal::status mpl3115a2::set_sea_pressure(float p_sea_level_pressure)
{
  // divide by 2 to convert to 2Pa per LSB
  auto two_pa = static_cast<std::uint16_t>(p_sea_level_pressure / 2.0f);
  auto two_pa_hi = static_cast<hal::byte>((two_pa & 0xFF00) >> 8);
  auto two_pa_lo = static_cast<hal::byte>(two_pa & 0x00FF);

  // write result to register
  std::array<hal::byte, 3> slp_payload = {
    bar_in_msb_r,
    two_pa_hi,  // msb
    two_pa_lo   // lsb
  };

  HAL_CHECK(
    hal::write(*m_i2c, device_address, slp_payload, hal::never_timeout()));

  return hal::success();
}

hal::status mpl3115a2::set_altitude_offset(int8_t p_offset)
{
  std::array<hal::byte, 2> offset_payload = { off_h_r, hal::byte(p_offset) };
  HAL_CHECK(
    hal::write(*m_i2c, device_address, offset_payload, hal::never_timeout()));

  return hal::success();
}

hal::result<mpl3115a2::temperature_read_t> mpl3115a2::read_temperature()
{
  constexpr float temp_conversion_factor = 256.0f;

  initiate_one_shot(m_i2c);

  poll_flag(m_i2c,
            { .address = status_r, .flag = status_tdr, .desired_state = true });

  // Read data from out_t_msb_r and out_t_lsb_r
  auto temp_buffer =
    HAL_CHECK(hal::write_then_read<2>(*m_i2c,
                                      device_address,
                                      std::array<hal::byte, 1>{ out_t_msb_r },
                                      hal::never_timeout()));

  auto temp_reading = (temp_buffer[0] << 8) | temp_buffer[1];
  return mpl3115a2::temperature_read_t{
    static_cast<float>(temp_reading) / temp_conversion_factor,
  };
}

hal::result<mpl3115a2::pressure_read_t> mpl3115a2::read_pressure()
{
  // Note: 64 -> Pa, 6400 -> kPa
  constexpr float pressure_conversion_factor = 64.0f;

  if (m_sensor_mode != mode::barometer) {
    set_mode(m_i2c, mode::barometer);
    m_sensor_mode = mode::barometer;
  }

  initiate_one_shot(m_i2c);

  poll_flag(m_i2c,
            { .address = status_r, .flag = status_pdr, .desired_state = true });

  // Read data from out_p_msb_r, out_p_csb_r, and out_p_lsb_r
  auto pres_buffer =
    HAL_CHECK(hal::write_then_read<3>(*m_i2c,
                                      device_address,
                                      std::array<hal::byte, 1>{ out_p_msb_r },
                                      hal::never_timeout()));

  uint32_t pressure_reading = uint32_t(pres_buffer[0]) << 16 |
                              uint32_t(pres_buffer[1]) << 8 |
                              uint32_t(pres_buffer[2]);

  return mpl3115a2::pressure_read_t{
    static_cast<float>(pressure_reading) / pressure_conversion_factor,
  };
}

hal::result<mpl3115a2::altitude_read_t> mpl3115a2::read_altitude()
{
  constexpr float altitude_conversion_factor = 65536.0f;

  if (m_sensor_mode != mode::altimeter) {
    set_mode(m_i2c, mode::altimeter);
    m_sensor_mode = mode::altimeter;
  }

  initiate_one_shot(m_i2c);

  poll_flag(m_i2c,
            { .address = status_r, .flag = status_pdr, .desired_state = true });

  // Read data from out_p_msb_r, out_p_csb_r, and out_p_lsb_r
  auto alt_buffer =
    HAL_CHECK(hal::write_then_read<3>(*m_i2c,
                                      device_address,
                                      std::array<hal::byte, 1>{ out_p_msb_r },
                                      hal::never_timeout()));

  int32_t alt_reading = int32_t(alt_buffer[0]) << 24 |
                        int32_t(alt_buffer[1]) << 16 |
                        int32_t(alt_buffer[2]) << 8;

  return mpl3115a2::altitude_read_t{
    static_cast<float>(alt_reading) / altitude_conversion_factor,
  };
}

}  // namespace hal::mpl
