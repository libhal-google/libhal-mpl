// Copyright 2023 Google LLC
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

#include <span>
#include <array>

#include <libhal/error.hpp>
#include <libhal/units.hpp>
#include <libhal-util/as_bytes.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal/i2c.hpp>
#include <libhal/timeout.hpp>
#include <libhal-util/serial.hpp>

namespace hal::mpl311 {

class mpl311
{
public:
    // default 7-bit I2C device address is 0b110'0000
    static constexpr hal::byte device_address = 0x60;

/** ---------- Typedefs ---------- **/
    typedef enum {
        BAROMETER_M = 0,
        ALTIMETER_M = 1,
    } mpl311_mode_t;

    struct temperature_read_t
    {
        celsius temperature;
    };

    struct pressure_read_t
    {
        float pressure; // Pascals (Pa)
    };

    struct altitude_read_t
    {
        meters altitude;
    };

/** ---------- Public Functions ---------- **/
    [[nodiscard]] static result<mpl311> create(hal::i2c& i2c);

    // Reset & Configure device
    hal::status driver_configure();

    [[nodiscard]] hal::result<temperature_read_t> read_temperature()
    {
        return t_read();
    }

    [[nodiscard]] hal::result<pressure_read_t> read_pressure()
    {
        return p_read();
    }

    [[nodiscard]] hal::result<altitude_read_t> read_altitude()
    {
        return a_read();
    }

    hal::status set_sea_pressure(float sea_level_pressure);
    
    hal::status set_altitude_offset(int8_t offset);

private:
    /// The I2C peripheral used for communication with the device.
    hal::i2c* m_i2c;

    // Variable to track current sensor mode to determine if CTRL_REG1 ALT flag needs to be set.
    mpl311_mode_t sensor_mode;

/** ---------- Private Functions ---------- **/
    /// @param i2c The I2C peripheral used for communication with the device.
    explicit mpl311(hal::i2c& p_i2c);

    // Set bit 7 (ALT - mode control) to the binary value of mode
    hal::status set_mode(mpl311_mode_t mode = BAROMETER_M);

    // Set specified bits in a register while maintaining its current state
    hal::status modify_reg_bits(hal::byte reg_addr, hal::byte bits_to_set);

    // Trigger a one-shot sample collection for the currently set mode.
    hal::status initiate_one_shot();

    // Check if the flag bit(s) are set in the specified register
    hal::status poll_flag(hal::byte reg, hal::byte flag, bool end_state);

    // Read and convert temperature values. Unit: celcius
    hal::result<temperature_read_t> t_read(); 

    // Read and convert pressure values. Unit: pascals
    hal::result<pressure_read_t> p_read();

    // Read and convert altitude values. Unit: meters
    hal::result<altitude_read_t> a_read();
};

}  // namespace hal::mpl311
