//! Copyright (c) M5Stack. All rights reserved.
//! Licensed under the MIT license.
//! See LICENSE file in the project root for full license information.

#include "common_header.h"

uint8_t config_save_countdown           = 0;
uint8_t localize_text_t::localize_index = 0;

constexpr const char* config_param_t::common_off_on_text[];

constexpr const char* config_param_t::misc_language_text[];

constexpr const uint8_t config_param_t::misc_cpuspeed_value[];
constexpr const uint8_t config_param_t::sens_refreshrate_value[];
constexpr const uint8_t config_param_t::sens_noisefilter_value[];
constexpr const uint8_t config_param_t::sens_monitorarea_value[];
constexpr const uint16_t config_param_t::cloud_interval_value[];
constexpr const uint8_t config_param_t::misc_brightness_value[];
constexpr const uint8_t config_param_t::misc_volume_value[];
constexpr const lgfx::IFont* config_param_t::misc_language_value[];
