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

/** ---------- MPL3115A2 Control Register Bits ---------- **/
    // Reset bit
    static constexpr hal::byte CTRL_REG1_RST = 0x04;
    // One-Shot trigger bit
    static constexpr hal::byte CTRL_REG1_OST = 0x02;
    // Altimeter-Barometer mode bit
    static constexpr hal::byte CTRL_REG1_ALT = 0x80;

/** ---------- mpl Oversample Values ---------- **/
    static constexpr hal::byte CTRL_REG1_OS32 = 0x28;
    static constexpr hal::byte CTRL_REG1_OS64 = 0x30;
    static constexpr hal::byte CTRL_REG1_OS128 = 0x38;

}