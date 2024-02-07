// Copyright 2024 Khalil Estell
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <libhal/i2c.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>

namespace hal::mpl {

class mpl3115a2
{
public:
  /* Keep track of the current set mode bit in ctrl_reg1 */
  enum class mode
  {
    barometer = 0,
    altimeter = 1,
  };

  struct temperature_read_t
  {
    celsius temperature;
  };

  struct pressure_read_t
  {
    float pressure;  // Pascals (Pa)
  };

  struct altitude_read_t
  {
    meters altitude;
  };

   /** ---------- MPLXOversample Values ---------- **/
  typedef enum mpl_os_rate
  {
    os1 = 0x00,   // 6ms
    os2 = 0x08,   // 10ms
    os4 = 0x10,   // 18ms
    os8 = 0x18,   // 34ms
    os16 = 0x20,  // 66ms
    os32 = 0x28,  // 130ms
    os64 = 0x30,  // 258ms
    os128 = 0x38  // 512ms  
  } mpl_os_rate;

  /**
   * @brief Initialization of MPLX device.
   *
   * This function performs the following steps during startup configuration:
   *   - Perform WHOAMI check
   *   - Trigger reset and wait for completion
   *   - Set altimeter mode
   *   - Set oversampling ratio to 2^128 (OS128)
   *   - Enable data ready events for pressure/altitude and temperature
   */
  [[nodiscard]] static result<mpl3115a2> create(hal::i2c& p_i2c, mpl_os_rate os_rate = mpl_os_rate::os32);

  /**
   * @brief Read pressure data from out_t_msb_r and out_t_lsb_r
   *        and perform temperature conversion to celsius.
   */
  [[nodiscard]] hal::result<temperature_read_t> read_temperature();

  /**
   * @brief Read pressure data from out_p_msb_r, out_p_csb_r, and out_p_lsb_r
   *        and perform pressure conversion to kilopascals.
   */
  [[nodiscard]] hal::result<pressure_read_t> read_pressure();

  /**
   * @brief Read altitude data from out_p_msb_r, out_p_csb_r, and out_p_lsb_r
   *        and perform altitude conversion to meters.
   */
  [[nodiscard]] hal::result<altitude_read_t> read_altitude();

  /**
   * @brief Set sea level pressure (Barometric input for altitude calculations)
   *        in bar_in_msb_r and bar_in_lsb_r registers
   * @param p_sea_level_pressure: Sea level pressure in Pascals.
   *        Default value on startup is 101,326 Pa.
   */
  hal::status set_sea_pressure(float p_sea_level_pressure);

  /**
   * @brief Set altitude offset in off_h_r
   * @param p_offset Offset value in meters, from -127 to 128
   */
  hal::status set_altitude_offset(int8_t p_offset);

  /* Maximum number of retries for polling operations. */
  static constexpr uint16_t default_max_polling_retries = 10000;

private:
  /**
   * @brief constructor for mpl objects
   * @param p_i2c The I2C peripheral used for communication with the device.
   */
  explicit mpl3115a2(hal::i2c& p_i2c);

  /* The I2C peripheral used for communication with the device. */
  hal::i2c* m_i2c;

  /* Variable to track current sensor mode to determine if CTRL_REG1 ALT flag
   * needs to be set. */
  mode m_sensor_mode = mode::barometer;
};

}  // namespace hal::mpl
