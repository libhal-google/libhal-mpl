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

#include <libhal/units.hpp>

namespace hal::mpl {
// default 7-bit I2C device address is 0b110'0000
static constexpr hal::byte device_address = 0x60;

/** ---------- MPL3115A2 Registers ---------- **/
// The address of the sensor status register - Alias for dr_status or f_status
static constexpr hal::byte status_r = 0x00;

// Bits 12-19 of 20-bit real-time Pressure sample register
static constexpr hal::byte out_p_msb_r = 0x01;
// Bits 4-11 of 20-bit real-time Pressure sample register
static constexpr hal::byte out_p_csb_r = 0x02;
// Bits 0-3 of 20-bit real-time Pressure sample register
static constexpr hal::byte out_p_lsb_r = 0x03;

// Bits 4-11 of 12-bit real-time Temperature sample register
static constexpr hal::byte out_t_msb_r = 0x04;
// Bits 0-3 of 12-bit real-time Temperature sample register
static constexpr hal::byte out_t_lsb_r = 0x05;

// Device identification register. Reset value is 0xC4
static constexpr hal::byte whoami_r = 0x0C;

// PT Data Configuration Register - data event flag config
static constexpr hal::byte pt_data_cfg_r = 0x13;

// Barometric input for Altitude calculation bits 8-15
static constexpr hal::byte bar_in_msb_r = 0x14;
// Barometric input for Altitude calculation bits 0-7
static constexpr hal::byte bar_in_lsb_r = 0x15;

// Control Register: Modes & Oversampling
static constexpr hal::byte ctrl_reg1 = 0x26;

// Altitude data user offset register
static constexpr hal::byte off_h_r = 0x2D;

/** ---------- MPL3115A2 Status Register Bits ---------- **/
// These status bits are set to 1 whenever corresponding data acquisition is
// completed. Status bits are cleared anytime OUT_*_MSB register is read.

// Temperature new data ready.
static constexpr hal::byte status_tdr = 0x02;
// Pressure/Altitude new data ready
static constexpr hal::byte status_pdr = 0x04;
// Pressure/Altitude OR Temperature data ready
static constexpr hal::byte status_ptdr = 0x08;

/** ---------- MPL3115A2 PT DATA Register Bits ---------- **/
// These bits must be configured at startup in order to
// enable the status_x flag functionality

// Data event flag enable on new Temperature data.
static constexpr hal::byte pt_data_cfg_tdefe = 0x01;
// Data event flag enable on new Pressure/Altitude data.
static constexpr hal::byte pt_data_cfg_pdefe = 0x02;
// Data ready event mode.
static constexpr hal::byte pt_data_cfg_drem = 0x04;

/** ---------- MPL3115A2 Control Register Bits ---------- **/
// Reset bit
static constexpr hal::byte ctrl_reg1_rst = 0x04;
// One-Shot trigger bit
static constexpr hal::byte ctrl_reg1_ost = 0x02;
// Altimeter-Barometer mode bit
static constexpr hal::byte ctrl_reg1_alt = 0x80;

}  // namespace hal::mpl