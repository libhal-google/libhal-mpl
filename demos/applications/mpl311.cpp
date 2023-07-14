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

#include <libhal-mpl311/mpl311.hpp>
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

  hal::print(console, "Demo Application Starting...\n\n");
  auto mpl311 = HAL_CHECK(hal::tmp::tmp102::mpl311(i2c, clock));

  while (true) {
    hal::delay(clock, 500ms);

    hal::print(console, "Reading temperature... \n");
    auto temperature = HAL_CHECK(mpl311.read_temperature()).temperature;
    hal::print<32>(console, "measured temperature = %f °C\n", temperature);

    hal::print(console, "Reading pressure... \n");
    auto pressure = HAL_CHECK(mpl311.read_pressure()).pressure;
    hal::print<32>(console, "measured pressure = %f Pa\n", pressure);

    hal::print(console, "Reading altitude... \n");
    auto altitude = HAL_CHECK(mpl311.read_altitude()).altitude;
    hal::print<32>(console, "measured altitude = %f m\n", altitude);
  }

  return hal::success();
}
