//! Copyright (c) M5Stack. All rights reserved.
//! Licensed under the MIT license.
//! See LICENSE file in the project root for full license information.

#pragma once

// #define DEBUG 1
#pragma GCC optimize("O3")

#include <cstdint>

#include "mlx90640.hpp"

namespace command_processor {
void setup(void);
bool loop(void);

bool addData(std::uint8_t value);
void closeData(void);
void prepareTxData(void);

void setRate(uint8_t rate);
void setFilter(uint8_t level);
void setEmissivity(uint8_t percent);
uint32_t getRecvCount(void);

m5::MLX90640_Class::temp_data_t* getTemperatureData(void);
}  // namespace command_processor
