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

#include <libhal-mpl/mpl3115a2.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include "../hardware_map.hpp"

hal::status application(hardware_map& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock;
  auto& console = *p_map.console;
  auto& i2c = *p_map.i2c;

  hal::print(console, "\n\nMPL3115A2 Demo Application Starting...\n");
  auto mpl_device = HAL_CHECK(hal::mpl::mpl3115a2::create(i2c));

  int8_t alt_offset = 0;
  mpl_device.set_altitude_offset(alt_offset);

  // Set sea level pressure to 30 Hg
  float slp = 101325;  // Default is 101325 Pa
  mpl_device.set_sea_pressure(slp);

  while (true) {
    hal::delay(clock, 500ms);

    auto temperature = HAL_CHECK(mpl_device.read_temperature()).temperature;
    hal::print<42>(console, "Measured temperature = %f °C\n", temperature);

    auto pressure = HAL_CHECK(mpl_device.read_pressure()).pressure;
    hal::print<42>(console, "Measured pressure = %f Pa\n", pressure);

    auto altitude = HAL_CHECK(mpl_device.read_altitude()).altitude;
    hal::print<42>(console, "Measured altitude = %f m\n\n", altitude);
  }

  return hal::success();
}
