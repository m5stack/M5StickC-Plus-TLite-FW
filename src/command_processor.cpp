//! Copyright (c) M5Stack. All rights reserved.
//! Licensed under the MIT license.
//! See LICENSE file in the project root for full license information.

#include "command_processor.hpp"

#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/rtc.h>

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_task_wdt.h>

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <algorithm>

#include <M5Unified.h>

#include "i2c_master.hpp"
#include "mlx90640.hpp"

namespace command_processor {

static constexpr const uint8_t noise_tbl[] = {
    0,  0,  0,  1,  2,  5,  8,  13, 20, 28, 39,  52,  67,  86,  107, 132, 160,
    0,  0,  0,  1,  3,  5,  9,  14, 20, 29, 39,  52,  68,  86,  108, 132, 160,
    0,  0,  1,  2,  3,  6,  9,  14, 21, 30, 41,  54,  69,  88,  109, 134, 162,
    1,  1,  1,  2,  4,  7,  11, 16, 23, 32, 42,  56,  72,  90,  112, 137, 165,
    1,  2,  2,  3,  5,  8,  12, 18, 25, 34, 45,  59,  75,  94,  116, 141, 170,
    3,  3,  4,  5,  7,  10, 15, 21, 28, 37, 49,  63,  79,  98,  121, 146, 175,
    4,  5,  6,  7,  10, 13, 18, 24, 32, 42, 54,  68,  85,  104, 127, 153, 182,
    7,  7,  8,  10, 13, 17, 22, 28, 37, 47, 59,  74,  91,  111, 134, 161, 191,
    11, 11, 12, 14, 17, 21, 27, 34, 42, 53, 66,  81,  99,  119, 143, 170, 200,
    15, 15, 17, 19, 22, 27, 33, 40, 49, 60, 74,  89,  108, 129, 153, 181, 212,
    21, 21, 22, 25, 29, 33, 40, 48, 57, 69, 83,  99,  118, 140, 165, 193, 225,
    27, 28, 29, 32, 36, 41, 48, 56, 67, 79, 93,  110, 130, 152, 178, 207, 239,
    35, 36, 38, 41, 45, 51, 58, 67, 77, 90, 105, 123, 143, 166, 193, 222, 255};

static m5::I2C_Master _i2c_in;
static m5::MLX90640_Class _mlx;

static constexpr size_t MLX_TEMP_ARRAY_SIZE      = 4;
static constexpr size_t MLX_FRAMEDATA_ARRAY_SIZE = 4;
static volatile int _idx_framedata               = -1;
static volatile int _idx_tempdata                = MLX_TEMP_ARRAY_SIZE - 1;
static uint16_t* _mlx_framedatas[MLX_FRAMEDATA_ARRAY_SIZE];
static m5::MLX90640_Class::temp_data_t* _mlx_tempdatas[MLX_TEMP_ARRAY_SIZE] = {
    nullptr};
static m5::MLX90640_Class::temp_data_t* _temp_data = nullptr;
// static int16_t* _diff_data;

static m5::MLX90640_Class::refresh_rate_t _refresh_rate;
static uint8_t _noise_filter = 8;
static uint8_t _emissivity   = 98;

/* clang-format off */
static inline volatile uint32_t* get_gpio_hi_reg(int_fast8_t pin) { return (pin & 32) ? &GPIO.out1_w1ts.val : &GPIO.out_w1ts; }
static inline volatile uint32_t* get_gpio_lo_reg(int_fast8_t pin) { return (pin & 32) ? &GPIO.out1_w1tc.val : &GPIO.out_w1tc; }
static inline bool gpio_in(int_fast8_t pin) { return ((pin & 32) ? GPIO.in1.data : GPIO.in) & (1 << (pin & 31)); }
static inline void gpio_hi(int_fast8_t pin) { *get_gpio_hi_reg(pin) = 1 << (pin & 31); }
static inline void gpio_lo(int_fast8_t pin) { *get_gpio_lo_reg(pin) = 1 << (pin & 31); }
/* clang-format on */

static void IRAM_ATTR mlxTask(void* main_handle) {
    gpio_num_t PIN_IN_SDA = GPIO_NUM_0;
    gpio_num_t PIN_IN_SCL = GPIO_NUM_26;
    i2c_port_t PORT_I2C   = I2C_NUM_0;
    switch (M5.getBoard()) {
        case m5::board_t::board_M5StackCore2:
            PIN_IN_SDA = (gpio_num_t)M5.Ex_I2C.getSDA();
            PIN_IN_SCL = (gpio_num_t)M5.Ex_I2C.getSCL();
            break;
        case m5::board_t::board_M5Stack:
            PIN_IN_SDA = (gpio_num_t)M5.In_I2C.getSDA();
            PIN_IN_SCL = (gpio_num_t)M5.In_I2C.getSCL();
            PORT_I2C   = I2C_NUM_1;
            break;
        default:
            break;
    }

    // running...
    size_t discard_count = 2;
    uint8_t error_count  = 255;
    for (;;) {
        if (error_count >= 128) {
            if (error_count == 128) {  // 強制的にSTOPコンディションを送信する
                ESP_EARLY_LOGD("mlxTask", "I2C force stop");
                gpio_config_t io_conf;
                io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
                io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
                io_conf.intr_type    = GPIO_INTR_DISABLE;
                io_conf.mode         = GPIO_MODE_OUTPUT;
                io_conf.pin_bit_mask = (uint64_t)1 << PIN_IN_SDA;
                gpio_config(&io_conf);
                io_conf.pin_bit_mask = (uint64_t)1 << PIN_IN_SCL;
                gpio_config(&io_conf);
                for (int i = 0; i < 20; ++i) {
                    vTaskDelay(1);
                    gpio_lo(PIN_IN_SCL);
                    vTaskDelay(1);
                    gpio_lo(PIN_IN_SDA);
                    vTaskDelay(1);
                    gpio_hi(PIN_IN_SCL);
                    vTaskDelay(1);
                    gpio_hi(PIN_IN_SDA);
                }
            }
            _i2c_in.release();
            // initialize sensor.
            _i2c_in.init(PORT_I2C, PIN_IN_SDA, PIN_IN_SCL);
            int retry = 16;

            while (!_mlx.init(&_i2c_in)) {
                ESP_EARLY_LOGD("mlxTask", "I2C int");
                vTaskDelay(100);
            }

            _mlx.setRate(_refresh_rate);

            error_count   = 0;
            discard_count = 2;
        }
        m5::MLX90640_Class::refresh_rate_t rate = _mlx.getRate();
        if (rate != _refresh_rate) {
            rate = _refresh_rate;
            _mlx.setRate(rate);
            // Discard twice because invalid data is obtained immediately after
            // refresh rate change.
            discard_count = 2;
        }
        int idx = (_idx_framedata + 1);
        if (idx >= MLX_FRAMEDATA_ARRAY_SIZE) {
            idx = 0;
        }
        auto recv = _mlx.readFrameData(_mlx_framedatas[idx]);
        ++error_count;
        if (recv) {
            error_count = 0;
            // ++recv_count;
            if (discard_count) {
                --discard_count;
            } else {
                _idx_framedata = idx;
                xTaskNotifyGive(main_handle);
            }
        } else {
            static constexpr const uint8_t delay_tbl[] = {32, 16, 8, 4,
                                                          2,  1,  1, 1};
            vTaskDelay(delay_tbl[rate]);
        }
    }
    vTaskDelete(nullptr);
}

m5::MLX90640_Class::temp_data_t* getTemperatureData(void) {
    return _mlx_tempdatas[_idx_tempdata];
}

void setRate(uint8_t rate) {
    _refresh_rate = (m5::MLX90640_Class::refresh_rate_t)rate;
}
void setFilter(uint8_t level) {
    _noise_filter = level;
}
void setEmissivity(uint8_t percent) {
    _emissivity = percent > 100 ? 100 : percent;
}

void setup(void) {
    for (int i = 0; i < MLX_FRAMEDATA_ARRAY_SIZE; ++i) {
        _mlx_framedatas[i] = (uint16_t*)heap_caps_malloc(
            m5::MLX90640_Class::FRAME_DATA_BYTES, MALLOC_CAP_DMA);
        memset(_mlx_framedatas[i], 0x2C, m5::MLX90640_Class::FRAME_DATA_BYTES);
    }

    xTaskCreatePinnedToCore(mlxTask, "mlxTask", 8192,
                            xTaskGetCurrentTaskHandle(), 20, nullptr,
                            APP_CPU_NUM);
    _refresh_rate = m5::MLX90640_Class::rate_32Hz;
    _noise_filter = 8;
    _emissivity   = 98;  // <- default : 98.0 %

    for (int i = 0; i < MLX_TEMP_ARRAY_SIZE; ++i) {
        _mlx_tempdatas[i] = (m5::MLX90640_Class::temp_data_t*)heap_caps_malloc(
            sizeof(m5::MLX90640_Class::temp_data_t), MALLOC_CAP_DMA);
        memset(_mlx_tempdatas[i], 0, sizeof(m5::MLX90640_Class::temp_data_t));
    }
}

bool IRAM_ATTR loop(void) {
    static int prev_idx_framedata = -1;
    if (prev_idx_framedata == _idx_framedata) return false;
    // if (prev_idx_framedata != _idx_framedata)

    {
#if DEBUG == 1
        {  // debug
            if (_idx_framedata !=
                ((prev_idx_framedata + 1) & (MLX_FRAMEDATA_ARRAY_SIZE - 1))) {
                ESP_LOGE(LOGNAME, "prev_idx_frame:%d  idx_frame:%d",
                         prev_idx_framedata, _idx_framedata);
            }
        }
#endif
        // prev_idx_framedata = _idx_framedata;
        prev_idx_framedata = prev_idx_framedata < MLX_FRAMEDATA_ARRAY_SIZE - 1
                                 ? prev_idx_framedata + 1
                                 : 0;
        // auto prev_temp_data = _mlx_tempdatas[_idx_tempdata];
        int idx =
            _idx_tempdata < MLX_TEMP_ARRAY_SIZE - 1 ? _idx_tempdata + 1 : 0;
        _temp_data = _mlx_tempdatas[idx];

        float emissivity = ((float)_emissivity) / 100.0f;
        _mlx.calcTempData(_mlx_framedatas[prev_idx_framedata], _temp_data,
                          emissivity);

        auto prev_temp_data = _mlx_tempdatas[(idx + MLX_TEMP_ARRAY_SIZE - 2) %
                                             MLX_TEMP_ARRAY_SIZE];

        static constexpr int16_t noise_filter_level[] = {181, 256,  362,  512,
                                                         724, 1024, 1448, 2048};
        int filter_value = noise_filter_level[_mlx.getRate()];
        int filter_level = (filter_value * (_noise_filter & 0xF)) >> 6;

        /// ノイズフィルタ処理
        if (filter_level) {
            bool subPage = _temp_data->subpage;
            for (size_t i = 0; i < 384; ++i) {
                int ilPattern   = (i >> 4) & 1;
                int pixelNumber = (i << 1) + ((ilPattern ^ subPage) & 1);
                /// (前回の温度と比較して一定以上の差がないと反応させない)
                int x = (pixelNumber & 31) - 15;
                if (x < 0) {
                    x = ~x;
                }
                int y = (pixelNumber >> 5) - 13;
                if (y < 0) {
                    y = ~y;
                }
                // 外周ピクセルほどノイズが多いため、ピクセル位置に応じてテーブルから補正係数を掛ける;
                int noise_filter =
                    (filter_level * (96 + noise_tbl[x + (y * 17)])) >> 8;

                int32_t temp = _temp_data->data[i];
                int diff     = temp - prev_temp_data->data[i];
                if (abs(diff) > noise_filter) {
                    temp += (diff < 0) ? noise_filter : -noise_filter;
                } else {
                    temp = prev_temp_data->data[i];
                }
                _temp_data->data[i] = temp;
            }
        }
        _idx_tempdata = idx;
    }
    return true;
}

}  // namespace command_processor