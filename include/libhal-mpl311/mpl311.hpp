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
#include <libhal-util/steady_clock.hpp>


namespace hal::mpl311 {
class mpl311
{
public:
    // default I2C device address is 0b110'0000
    static constexpr hal::byte device_address = 0x60;

/** ---------- MPL3115A2 Registers ---------- **/
    // The address of the sensor status register - Alias for DR_STATUS or F_STATUS
    static constexpr hal::byte STATUS_R = 0x00;

    // Bits 12-19 of 20-bit real-time Pressure sample register
    static constexpr hal::byte OUT_P_MSB_R = 0x01;
    // Bits 4-11 of 20-bit real-time Pressure sample register
    static constexpr hal::byte OUT_P_CSB_R = 0x02;
    // Bits 0-3 of 20-bit real-time Pressure sample register
    static constexpr hal::byte OUT_P_LSB_R = 0x03;

    // Bits 4-11 of 12-bit real-time Temperature sample register
    static constexpr hal::byte OUT_T_MSB_R = 0x04;
    // Bits 0-3 of 12-bit real-time Temperature sample register
    static constexpr hal::byte OUT_T_LSB_R = 0x05;

    // Device identification register. Reset value is 0xC4
    static constexpr hal::byte WHOAMI_R = 0x0C;

    // PT Data Configuration Register - data event flag config
    static constexpr hal::byte PT_DATA_CFG_R = 0x13;

    // Barometric input for Altitude calculation bits 8-15
    static constexpr hal::byte BAR_IN_MSB_R = 0x14;
    // Barometric input for Altitude calculation bits 0-7
    static constexpr hal::byte BAR_IN_LSB_R = 0x15;

    // Control Register: Modes & Oversampling
    static constexpr hal::byte CTRL_REG1 = 0x26;
    
    // Altitude data user offset register
    static constexpr hal::byte OFF_H_R = 0x2D;

/** ---------- MPL3115A2 Status Register Bits ---------- **/
    // These status bits are set to 1 whenever corresponding data acquisition is completed. 
    // Status bits are cleared anytime OUT_*_MSB register is read.

    // Temperature new data ready. 
    static constexpr hal::byte STATUS_TDR = 0x02;
    // Pressure/Altitude new data ready
    static constexpr hal::byte STATUS_PDR = 0x04;
    // Pressure/Altitude OR Temperature data ready
    static constexpr hal::byte STATUS_PTDR = 0x08;

/** ---------- MPL3115A2 PT DATA Register Bits ---------- **/
    // These bits must be configured at startup in order to 
    // enable the STATUS_X flag functionality

    // Data event flag enable on new Temperature data.
    static constexpr hal::byte PT_DATA_CFG_TDEFE = 0x01;
    // Data event flag enable on new Pressure/Altitude data. 
    static constexpr hal::byte PT_DATA_CFG_PDEFE = 0x02;
    // Data ready event mode.
    static constexpr hal::byte PT_DATA_CFG_DREM = 0x04;

/** ---------- MPL311 Control Register Bits ---------- **/
    // Reset bit
    static constexpr hal::byte CTRL_REG1_RST = 0x04;
    // One-Shot trigger bit
    static constexpr hal::byte CTRL_REG1_OST = 0x02;
    // Altimeter-Barometer mode bit
    static constexpr hal::byte CTRL_REG1_ALT = 0x80;

/** ---------- MPL311 Oversample Values ---------- **/
    static constexpr hal::byte CTRL_REG1_OS32 = 0x28;
    static constexpr hal::byte CTRL_REG1_OS64 = 0x30;
    static constexpr hal::byte CTRL_REG1_OS128 = 0x38;

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
        float pressure; // kilopascals (KPa)
    };

    struct altitude_read_t
    {
        meters altitude;
    };

/** ---------- Public Functions ---------- **/
    static result<mpl311> create(hal::i2c& i2c, hal::steady_clock& clk)
    {
        return mpl311(i2c, clk);
    }

    // Reset & Configure device
    hal::status begin();

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

    hal::steady_clock* m_clk;

    // Variable to track current sensor mode to determine if CTRL_REG1 ALT flag needs to be set.
    mpl311_mode_t sensor_mode;

    /// @param i2c The I2C peripheral used for communication with the device.
    explicit constexpr mpl311(hal::i2c& p_i2c, hal::steady_clock& p_clk):
        m_i2c(&p_i2c), 
        m_clk(&p_clk) 
    {}

    // Set bit 7 (ALT - mode control) to the binary value of mode
    hal::status set_mode(mpl311_mode_t mode = BAROMETER_M);

    // Set specified bits in a register while maintaining its current state
    hal::status modify_reg_bits(hal::byte reg_addr, hal::byte bits_to_set);

    // Trigger a one-shot sample collection for the currently set mode.
    hal::status initiate_one_shot();

    // Check if the flag bit(s) are set in the STATUS_R register
    hal::result<bool> check_data_ready_flag(hal::byte flag);

    // Read and convert temperature values. Unit: celcius
    hal::result<temperature_read_t> t_read(); 

    // Read and convert pressure values. Unit: pascals
    hal::result<pressure_read_t> p_read();

    // Read and convert altitude values. Unit: meters
    hal::result<altitude_read_t> a_read();
};
}  // namespace hal::mpl311
