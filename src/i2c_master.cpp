//! Copyright (c) M5Stack. All rights reserved.
//! Licensed under the MIT license.
//! See LICENSE file in the project root for full license information.

#include "i2c_master.hpp"

#include <driver/rtc_io.h>
#include <driver/periph_ctrl.h>
#include <esp_log.h>
#include <soc/i2c_reg.h>
#include <soc/rtc.h>
#include <soc/soc.h>

#include <algorithm>
#include <cstring>

namespace m5 {
#if !defined(I2C_ACK_ERR_INT_RAW_M)
#define I2C_ACK_ERR_INT_RAW_M I2C_NACK_INT_RAW_M
#endif
__attribute__((unused)) static inline unsigned long millis(void) {
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}
__attribute__((unused)) static inline unsigned long micros(void) {
    return (unsigned long)(esp_timer_get_time());
}

#if __has_include(<soc/i2c_periph.h>)
#include <soc/i2c_periph.h>
static inline periph_module_t getPeriphModule(i2c_port_t num) {
    return i2c_periph_signal[num].module;
}
static inline std::uint8_t getPeriphIntSource(i2c_port_t num) {
    return i2c_periph_signal[num].irq;
}
#else
static inline periph_module_t getPeriphModule(i2c_port_t num) {
    return num == 0 ? PERIPH_I2C0_MODULE : PERIPH_I2C1_MODULE;
}
static inline std::uint8_t getPeriphIntSource(i2c_port_t num) {
    return num == 0 ? ETS_I2C_EXT0_INTR_SOURCE : ETS_I2C_EXT1_INTR_SOURCE;
}
#endif

#if SOC_I2C_NUM == 1
static inline i2c_dev_t *IRAM_ATTR getDev(i2c_port_t num) {
    return &I2C0;
}
#else
static __attribute__((always_inline)) inline i2c_dev_t *IRAM_ATTR
getDev(i2c_port_t num) {
    return num == 0 ? &I2C0 : &I2C1;
}
#endif

#if defined(CONFIG_IDF_TARGET_ESP32C3)

static i2c_dev_t *get_i2c_dev(int num) {
    return &I2C0;
}
static void updateDev(i2c_dev_t *dev) {
    dev->ctr.conf_upgate = 1;
}
static volatile uint32_t *getFifoAddr(int num) {
    return &I2C0.fifo_data.val;
}

static constexpr int i2c_cmd_start = 6;
static constexpr int i2c_cmd_write = 1;
static constexpr int i2c_cmd_read  = 3;
static constexpr int i2c_cmd_stop  = 2;
static constexpr int i2c_cmd_end   = 4;

#else

static __attribute__((always_inline)) inline i2c_dev_t *get_i2c_dev(int num) {
    return num == 0 ? &I2C0 : &I2C1;
}

static __attribute__((always_inline)) inline void updateDev(i2c_dev_t *dev) {
}

static __attribute__((always_inline)) inline volatile uint32_t *getFifoAddr(
    int num) {
    return (volatile uint32_t *)((num == 0) ? 0x6001301c : 0x6002701c);
}
static __attribute__((always_inline)) inline std::uint32_t IRAM_ATTR
getRxFifoCount(i2c_dev_t *dev) {
    return dev->status_reg.rx_fifo_cnt;
}

static constexpr int i2c_cmd_start = 0;
static constexpr int i2c_cmd_write = 1;
static constexpr int i2c_cmd_read  = 2;
static constexpr int i2c_cmd_stop  = 3;
static constexpr int i2c_cmd_end   = 4;

#endif

static void IRAM_ATTR i2c_set_cmd(i2c_dev_t *dev, uint8_t index,
                                  uint8_t op_code, uint8_t byte_num,
                                  bool ack_value = false) {
    uint32_t cmd_val = byte_num |
                       ((op_code == i2c_cmd_write || op_code == i2c_cmd_stop)
                            ? 0x100
                            : 0)  // writeおよびstop時はACK_ENを有効にする;
                       | ack_value << 10  // 受信時ACK応答値
                       | op_code << 11;
    dev->command[index].val = cmd_val;
}

static constexpr int I2C_7BIT_ADDR_MIN  = 0x08;
static constexpr int I2C_7BIT_ADDR_MAX  = 0x77;
static constexpr int I2C_10BIT_ADDR_MAX = 1023;

void IRAM_ATTR I2C_Master::save_reg(void) {
    auto i2c_dev     = get_i2c_dev(_i2c_port);
    scl_high_period  = i2c_dev->scl_high_period.val;
    scl_low_period   = i2c_dev->scl_low_period.val;
    scl_start_hold   = i2c_dev->scl_start_hold.val;
    scl_rstart_setup = i2c_dev->scl_rstart_setup.val;
    scl_stop_hold    = i2c_dev->scl_stop_hold.val;
    scl_stop_setup   = i2c_dev->scl_stop_setup.val;
    sda_hold         = i2c_dev->sda_hold.val;
    sda_sample       = i2c_dev->sda_sample.val;
    fifo_conf        = i2c_dev->fifo_conf.val;
    timeout          = i2c_dev->timeout.val;
#if defined(I2C_FILTER_CFG_REG)
    filter_cfg = dev->filter_cfg.val;
#else
    scl_filter                  = i2c_dev->scl_filter_cfg.val;
    sda_filter                  = i2c_dev->sda_filter_cfg.val;
#endif
}

void IRAM_ATTR I2C_Master::load_reg(void) {
    auto i2c_dev                  = get_i2c_dev(_i2c_port);
    i2c_dev->scl_high_period.val  = scl_high_period;
    i2c_dev->scl_low_period.val   = scl_low_period;
    i2c_dev->scl_start_hold.val   = scl_start_hold;
    i2c_dev->scl_rstart_setup.val = scl_rstart_setup;
    i2c_dev->scl_stop_hold.val    = scl_stop_hold;
    i2c_dev->scl_stop_setup.val   = scl_stop_setup;
    i2c_dev->sda_hold.val         = sda_hold;
    i2c_dev->sda_sample.val       = sda_sample;
    i2c_dev->fifo_conf.val        = fifo_conf;
    i2c_dev->timeout.val          = timeout;
#if defined(I2C_FILTER_CFG_REG)
    dev->filter_cfg.val = filter_cfg;
#else
    i2c_dev->scl_filter_cfg.val = scl_filter;
    i2c_dev->sda_filter_cfg.val = sda_filter;
#endif
}

void IRAM_ATTR I2C_Master::i2c_stop(void) {
    static constexpr int I2C_CLR_BUS_HALF_PERIOD_US =
        5;  // use standard 100kHz data rate
    static constexpr int I2C_CLR_BUS_SCL_NUM = 9;

    gpio_set_level(_pin_sda, 1);
    gpio_set_direction(_pin_sda, GPIO_MODE_INPUT_OUTPUT_OD);

    gpio_set_level(_pin_scl, 1);
    gpio_set_direction(_pin_scl, GPIO_MODE_OUTPUT_OD);

    auto mod = getPeriphModule(_i2c_port);
    // ESP-IDF環境でperiph_module_disableを使うと、後でenableできなくなる問題が起きたためコメントアウト;
    // periph_module_disable(mod);
    gpio_set_level(_pin_scl, 0);

    // SDAがHIGHになるまでクロック送出しながら待機する。;
    int i = 0;
    while (!gpio_get_level(_pin_sda) && (i++ < I2C_CLR_BUS_SCL_NUM)) {
        ets_delay_us(I2C_CLR_BUS_HALF_PERIOD_US);
        gpio_set_level(_pin_scl, 1);
        ets_delay_us(I2C_CLR_BUS_HALF_PERIOD_US);
        gpio_set_level(_pin_scl, 0);
    }
    gpio_set_level(_pin_sda, 0);  // setup for STOP
    periph_module_enable(mod);
    gpio_set_level(_pin_scl, 1);
    periph_module_reset(mod);
    // gpio_set_level(pin_sda, 1); // STOP, SDA low -> high while SCL is HIGH
    i2c_set_pin((i2c_port_t)_i2c_port, _pin_sda, _pin_scl,
                gpio_pullup_t::GPIO_PULLUP_ENABLE,
                gpio_pullup_t::GPIO_PULLUP_ENABLE, I2C_MODE_MASTER);
}

bool IRAM_ATTR I2C_Master::i2c_wait(bool flg_stop) {
    auto i2c_dev = get_i2c_dev(_i2c_port);
    if (i2c_dev == nullptr) {
        return false;
    }
    if (_state == state_t::state_error) {
        return false;
    }
    bool res = true;
    if (_state == state_t::state_disconnect) {
        return res;
    }

    typeof(i2c_dev->int_raw) int_raw;
    static constexpr uint32_t intmask = I2C_ACK_ERR_INT_RAW_M |
                                        I2C_END_DETECT_INT_RAW_M |
                                        I2C_ARBITRATION_LOST_INT_RAW_M;
    if (_wait_ack) {
        int_raw.val = i2c_dev->int_raw.val;
        if (!(int_raw.val & intmask)) {
            uint32_t us = micros();
#if defined(CONFIG_IDF_TARGET_ESP32C3)
            uint32_t us_limit = (dev->scl_high_period.period +
                                 dev->scl_low_period.period + 16) *
                                (1 + dev->sr.tx_fifo_cnt);
#else
            uint32_t us_limit = (i2c_dev->scl_high_period.period +
                                 i2c_dev->scl_low_period.period + 20) *
                                (2 + i2c_dev->status_reg.tx_fifo_cnt);
#endif
            // static uint32_t prev = -1;
            // if (prev != us_limit)
            // {
            //   prev = us_limit;
            //   ESP_LOGW("us_limit","us_limit:%d", us_limit);
            // }
            do {
                int_raw.val = i2c_dev->int_raw.val;
            } while (!(int_raw.val & intmask) && ((micros() - us) <= us_limit));
        }
        i2c_dev->int_clr.val = int_raw.val;
#if !defined(CONFIG_IDF_TARGET) || defined(CONFIG_IDF_TARGET_ESP32)
        if (!int_raw.end_detect || int_raw.ack_err)
#else
        if (!int_raw.end_detect || int_raw.nack)
#endif
        {
            res    = false;
            _state = state_t::state_error;
        }
    }

    if (flg_stop || !res) {
        if (_state == state_t::state_read ||
            !int_raw.end_detect) {  // force stop
            i2c_stop();
        } else {
            i2c_set_cmd(i2c_dev, 0, i2c_cmd_stop, 0);
            i2c_set_cmd(i2c_dev, 1, i2c_cmd_end, 0);
            i2c_dev->ctr.trans_start = 1;
            static constexpr uint32_t intmask =
                I2C_ACK_ERR_INT_RAW_M | I2C_TIME_OUT_INT_RAW_M |
                I2C_END_DETECT_INT_RAW_M | I2C_ARBITRATION_LOST_INT_RAW_M |
                I2C_TRANS_COMPLETE_INT_RAW_M;
            uint32_t ms = millis();
            taskYIELD();
            while (!(i2c_dev->int_raw.val & intmask) && ((millis() - ms) < 14))
                ;
#if !defined(CONFIG_IDF_TARGET) || defined(CONFIG_IDF_TARGET_ESP32)
            if (res && i2c_dev->int_raw.ack_err)
#else
            if (res && dev->int_raw.nack)
#endif
            {
                res = false;
            }
        }
        load_reg();
        if (res) {
            _state = state_t::state_disconnect;
        }
    }
    _wait_ack = false;
    return res;
}

bool IRAM_ATTR I2C_Master::_readword_inner(i2c_dev_t *dev, uint8_t *data,
                                           size_t length, bool last_nack) {
    size_t len = ((length - 1) & 127) + 1;
    length -= len;
    auto res = i2c_wait(false);
    if (!res) {
        ESP_LOGW("I2C_Master", "readWords error : ack wait");
        return false;
    }

    _isr_recv_buf        = data;
    _isr_recv_remain_len = length;
    _isr_mode            = isr_mode_t::isr_readword;

    i2c_set_cmd(dev, 0, i2c_cmd_read, (len << 1) - 1);
    i2c_set_cmd(dev, 1, i2c_cmd_read, 1, length == 0 && last_nack);
    i2c_set_cmd(dev, 2, i2c_cmd_end, 0);
    updateDev(dev);

    static constexpr uint32_t intmask =
        I2C_ACK_ERR_INT_RAW_M | I2C_TIME_OUT_INT_RAW_M |
        I2C_RXFIFO_FULL_INT_RAW_M | I2C_END_DETECT_INT_RAW_M |
        I2C_ARBITRATION_LOST_INT_RAW_M;
    dev->int_clr.val     = ~0u;
    dev->int_ena.val     = intmask;
    dev->ctr.trans_start = 1;
    return true;
}

void IRAM_ATTR I2C_Master::_isr_handler(void *arg) {
    auto me = (I2C_Master *)arg;
    if (me->_isr_mode == isr_mode_t::isr_nojob) {
        return;
    }

    bool result = true;
    auto dev    = getDev(me->_i2c_port);
    do {
        typeof(dev->int_status) int_sts;
        int_sts.val      = dev->int_status.val;
        dev->int_clr.val = int_sts.val;

        if (me->_isr_mode == isr_mode_t::isr_readword) {
            auto fifo_addr     = getFifoAddr(me->_i2c_port);
            auto dst           = me->_isr_recv_buf;
            auto recv_done_len = me->_isr_recv_done_len;

            while (1 < getRxFifoCount(dev)) {
                auto tmp = *fifo_addr;
                *dst++   = *fifo_addr;
                *dst++   = tmp;
                ++recv_done_len;
            }
            me->_isr_recv_buf      = dst;
            me->_isr_recv_done_len = recv_done_len;
            if (int_sts.ack_err || int_sts.time_out ||
                int_sts.arbitration_lost) {
                me->_isr_result = false;
            }
            if (int_sts.end_detect) {
                if (me->_isr_recv_remain_len) {
                    me->_isr_result = me->_readword_inner(
                        dev, dst, me->_isr_recv_remain_len, true);
                } else {
                    if (me->_isr_semaphore) {
                        BaseType_t xHigherPriorityTaskWoken = pdTRUE;
                        xSemaphoreGiveFromISR(me->_isr_semaphore,
                                              &xHigherPriorityTaskWoken);
                        portYIELD_FROM_ISR();
                    }
                }
            }
        }
    } while (dev->int_status.val);
}

bool I2C_Master::init(int i2c_port, int pin_sda, int pin_scl) {
    if (i2c_port >= I2C_NUM_MAX) {
        return false;
    }
    this->_i2c_port = i2c_port;
    this->save_reg();
    release();
    this->_pin_scl = (gpio_num_t)pin_scl;
    this->_pin_sda = (gpio_num_t)pin_sda;
    i2c_stop();
    this->load_reg();

    esp_intr_alloc(getPeriphIntSource(i2c_port),
                   ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3, _isr_handler,
                   this, nullptr);

    return true;
}

bool I2C_Master::release(void) {
    gpio_reset_pin((gpio_num_t)_pin_sda);
    gpio_reset_pin((gpio_num_t)_pin_scl);
    return true;
}

bool I2C_Master::setFreq(uint32_t freq) {
    auto i2c_dev = get_i2c_dev(_i2c_port);
    if (i2c_dev == nullptr) return false;

    _freq                                   = freq;
    static constexpr uint32_t MIN_I2C_CYCLE = 40;  // 35;
#if defined(CONFIG_IDF_TARGET_ESP32C3)
    uint32_t src_clock = 40 * 1000 * 1000;  // XTAL clock
#else
    rtc_cpu_freq_config_t cpu_freq_conf;
    rtc_clk_cpu_freq_get_config(&cpu_freq_conf);
    uint32_t src_clock = 80 * 1000 * 1000;
    if (cpu_freq_conf.freq_mhz < 80) {
        src_clock =
            (cpu_freq_conf.source_freq_mhz * 1000000) / cpu_freq_conf.div;
    }
#endif

    auto cycle = std::min<uint32_t>(
        32767u, std::max(MIN_I2C_CYCLE, (src_clock / (freq + 1) + 1)));
    freq = src_clock / cycle;

#if defined(CONFIG_IDF_TARGET_ESP32S2)
    dev->ctr.ref_always_on = 1;
#endif

#if defined(I2C_FILTER_CFG_REG)
    dev->filter_cfg.scl_en    = cycle > 64;
    dev->filter_cfg.scl_thres = 0;
    dev->filter_cfg.sda_en    = cycle > 64;
    dev->filter_cfg.sda_thres = 0;

    uint32_t scl_high_offset =
        (dev->filter_cfg.scl_en ? (dev->filter_cfg.scl_thres <= 2
                                       ? 8
                                       : (6 + dev->filter_cfg.scl_thres))
                                : 7);
    dev->clk_conf.sclk_sel = 0;
#else
    i2c_dev->scl_filter_cfg.en    = cycle > 64;
    i2c_dev->scl_filter_cfg.thres = 0;
    i2c_dev->sda_filter_cfg.en    = cycle > 64;
    i2c_dev->sda_filter_cfg.thres = 0;
    /// ESP32 TRM page 286  Table 57: SCL Frequency Configuration
    uint32_t scl_high_offset =
        (i2c_dev->scl_filter_cfg.en
               ? (i2c_dev->scl_filter_cfg.thres <= 2
                      ? 8
                      : (6 + i2c_dev->scl_filter_cfg.thres))
               : 7);

#endif

    uint32_t period_total    = cycle - scl_high_offset - 1;
    uint32_t scl_high_period = std::max<uint32_t>(18, (period_total - 10) >> 1);
    uint32_t scl_low_period  = period_total - scl_high_period;

    i2c_dev->scl_high_period.period = scl_high_period;
    i2c_dev->scl_low_period.period  = scl_low_period;
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32C3)
    dev->scl_high_period.scl_wait_high_period = scl_high_period >> 2;
#endif
    i2c_dev->sda_hold.time =
        std::min(1023, (i2c_dev->scl_high_period.period >> 1));
    i2c_dev->sda_sample.time =
        std::min(1023, (i2c_dev->scl_low_period.period >> 1));

    if (freq > 400000) {
        cycle = cycle * freq / 400000;
    } else if (cycle > ((1 << 10) - 1)) {
        cycle = (1 << 10) - 1;
    }
    // the clock num after the STOP bit's posedge
    i2c_dev->scl_stop_hold.time = cycle << 1;
    // the clock num between the posedge of SCL and the posedge of SDA
    i2c_dev->scl_stop_setup.time = cycle;
    // the clock num between the negedge of SDA and
    // the negedge of SCL for start mark
    i2c_dev->scl_start_hold.time = cycle;
    // the clock num between the posedge of SCL and
    // the negedge of SDA for restart mark
    i2c_dev->scl_rstart_setup.time = cycle;

    return true;
}

bool I2C_Master::restart(int i2c_addr, bool read, uint32_t freq) {
    auto i2c_dev = get_i2c_dev(_i2c_port);
    if (i2c_dev == nullptr) {
        return false;
    }
    if (i2c_addr < I2C_7BIT_ADDR_MIN || i2c_addr > I2C_10BIT_ADDR_MAX)
        return false;

    auto res = i2c_wait(false);
    if (!res) return res;

    auto fifo_addr = getFifoAddr(_i2c_port);
    i2c_set_cmd(i2c_dev, 0, i2c_cmd_start, 0);
    i2c_set_cmd(i2c_dev, 2, i2c_cmd_end, 0);
    if (i2c_addr <= I2C_7BIT_ADDR_MAX) {  // 7bitアドレスの場合;
        *fifo_addr =
            i2c_addr << 1 | (read ? I2C_MASTER_READ : I2C_MASTER_WRITE);
        i2c_set_cmd(i2c_dev, 1, i2c_cmd_write, 1);
    } else {  // 10bitアドレスの場合;
        *fifo_addr = 0xF0 | (i2c_addr >> 8) << 1 | I2C_MASTER_WRITE;
        *fifo_addr = i2c_addr;
        i2c_set_cmd(i2c_dev, 1, i2c_cmd_write, 2);
        if (read) {  // 10bitアドレスのread要求の場合;
            *fifo_addr = 0xF0 | (i2c_addr >> 8) << 1 | I2C_MASTER_READ;
            i2c_set_cmd(i2c_dev, 2, i2c_cmd_start, 0);
            i2c_set_cmd(i2c_dev, 3, i2c_cmd_read, 1);
            i2c_set_cmd(i2c_dev, 4, i2c_cmd_end, 0);
        }
    }

    if (_state == state_t::state_disconnect || _freq != freq) {
        setFreq(freq);
    }

    updateDev(i2c_dev);
    i2c_dev->int_clr.val     = 0x1FFFF;
    i2c_dev->ctr.trans_start = 1;
    _state    = read ? state_t::state_read : state_t::state_write;
    _wait_ack = true;
    return res;
}

bool I2C_Master::start(int i2c_addr, bool read, uint32_t freq) {
    auto i2c_dev = get_i2c_dev(_i2c_port);
    if (i2c_dev == nullptr) return false;

    save_reg();

#if defined(CONFIG_IDF_TARGET_ESP32C3)
    if (dev->sr.bus_busy)
#else
    if (i2c_dev->status_reg.bus_busy)
#endif
    {
        auto ms = micros();
        do {
            taskYIELD();
        }
#if defined(CONFIG_IDF_TARGET_ESP32C3)
        while (dev->sr.bus_busy && micros() - ms < 128);
#else
        while (i2c_dev->status_reg.bus_busy && micros() - ms < 128);
#endif
    }

#if SOC_I2C_SUPPORT_HW_FSM_RST
    dev->ctr.fsm_rst = 1;
#endif

#if defined(CONFIG_IDF_TARGET_ESP32C3)
    dev->timeout.time_out_value = 31;
    dev->timeout.time_out_en    = 1;
#else
    i2c_dev->timeout.tout = 0xFFFFF;  // max 13ms
#endif
    i2c_dev->int_ena.val = 0;
    // ---------- i2c_ll_master_init
    typeof(i2c_dev->ctr) ctrl_reg;
    ctrl_reg.val           = 0;
    ctrl_reg.ms_mode       = 1;  // master mode
    ctrl_reg.clk_en        = 1;
    ctrl_reg.sda_force_out = 1;
    ctrl_reg.scl_force_out = 1;
    i2c_dev->ctr.val       = ctrl_reg.val;
    // ---------- i2c_ll_master_init
    typeof(i2c_dev->fifo_conf) fifo_conf_reg;
    fifo_conf_reg.val         = 0;
    fifo_conf_reg.tx_fifo_rst = 1;
    fifo_conf_reg.rx_fifo_rst = 1;
    i2c_dev->fifo_conf.val    = fifo_conf_reg.val;

    fifo_conf_reg.val                 = 0;
    fifo_conf_reg.rx_fifo_full_thrhd  = 24;
    fifo_conf_reg.tx_fifo_empty_thrhd = 4;
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32C3)
    fifo_conf_reg.fifo_prt_en = 1;
#endif
    i2c_dev->fifo_conf.val = fifo_conf_reg.val;

    _state = state_t::state_disconnect;

    return restart(i2c_addr, read, freq);
}

bool I2C_Master::stop(void) {
    return i2c_wait(true);
}

bool I2C_Master::writeBytes(const uint8_t *data, size_t length) {
    auto i2c_dev = get_i2c_dev(_i2c_port);
    if (i2c_dev == nullptr) {
        return false;
    }
    if (_state == state_t::state_error) {
        return false;
    }
    if (_state == state_t::state_read) {
        return false;
    }
    if (!length) {
        return true;
    }

    static constexpr int txfifo_limit = 32;
    auto dev                          = get_i2c_dev(_i2c_port);
    auto fifo_addr                    = getFifoAddr(_i2c_port);
    size_t len                        = ((length - 1) & (txfifo_limit - 1)) + 1;
    do {
        if (!i2c_wait(false)) {
            ESP_LOGW("I2C_Master", "writeBytes error : ack wait");
            return false;
        }
        size_t idx = 0;
        do {
            *fifo_addr = data[idx];
        } while (++idx != len);
        i2c_set_cmd(dev, 0, i2c_cmd_write, len);
        i2c_set_cmd(dev, 1, i2c_cmd_end, 0);
        updateDev(dev);
        dev->ctr.trans_start = 1;
        _wait_ack            = true;
        data += len;
        length -= len;
        len = txfifo_limit;
    } while (length);
    return true;
}

bool I2C_Master::writeWords(const uint16_t *data, size_t length) {
    auto i2c_dev = get_i2c_dev(_i2c_port);
    if (i2c_dev == nullptr) {
        return false;
    }
    if (_state == state_t::state_error) {
        return false;
    }
    if (_state == state_t::state_read) {
        return false;
    }
    if (!length) {
        return true;
    }

    static constexpr int txfifo_limit = 16;
    auto dev                          = get_i2c_dev(_i2c_port);
    auto fifo_addr                    = getFifoAddr(_i2c_port);
    size_t len                        = ((length - 1) & (txfifo_limit - 1)) + 1;
    do {
        if (!i2c_wait(false)) {
            ESP_LOGW("I2C_Master", "writeWords error : ack wait");
            return false;
        }
        size_t idx = 0;
        do {
            *fifo_addr = data[idx] >> 8;
            *fifo_addr = data[idx] & 0xFF;
        } while (++idx != len);
        i2c_set_cmd(dev, 0, i2c_cmd_write, len << 1);
        i2c_set_cmd(dev, 1, i2c_cmd_end, 0);
        updateDev(dev);
        dev->ctr.trans_start = 1;
        _wait_ack            = true;
        data += len;
        length -= len;
        len = txfifo_limit;
    } while (length);
    return true;
}
/*
bool I2C_Master::readBytes(uint8_t *readdata, size_t length, bool last_nack) {
    auto i2c_dev = get_i2c_dev(_i2c_port);
    if (i2c_dev == nullptr) {
        return false;
    }
    if (_state == state_t::state_error) {
        return false;
    }
    if (_state == state_t::state_write) {
        return false;
    }
    if (!length) {
        return true;
    }
    bool res = true;

    static constexpr uint32_t intmask =
        I2C_ACK_ERR_INT_RAW_M | I2C_TIME_OUT_INT_RAW_M |
        I2C_END_DETECT_INT_RAW_M | I2C_ARBITRATION_LOST_INT_RAW_M;
    auto fifo_addr = getFifoAddr(_i2c_port);
    auto dev       = get_i2c_dev(_i2c_port);
    size_t len     = 0;

    uint32_t us_limit =
        (dev->scl_high_period.period + dev->scl_low_period.period + 16);
    do {
        len = ((length - 1) & 31) + 1;
        length -= len;
        res = i2c_wait(false);
        if (!res) {
            ESP_LOGW("I2C_Master", "readBytes error : ack wait");
            break;
        }
        if (length == 0 && last_nack) {
            if (len == 1) {
                i2c_set_cmd(dev, 0, i2c_cmd_read, len, true);
                i2c_set_cmd(dev, 1, i2c_cmd_end, 0);
            } else {
                i2c_set_cmd(dev, 0, i2c_cmd_read, len - 1);
                i2c_set_cmd(dev, 1, i2c_cmd_read, 1, true);
                i2c_set_cmd(dev, 2, i2c_cmd_end, 0);
            }
        } else {
            i2c_set_cmd(dev, 0, i2c_cmd_read, len);
            i2c_set_cmd(dev, 1, i2c_cmd_end, 0);
        }
        dev->int_clr.val     = intmask;
        dev->ctr.trans_start = 1;
        do {
            uint32_t us = micros();
#if defined(CONFIG_IDF_TARGET_ESP32C3)
            while (0 == dev->sr.rx_fifo_cnt && !(dev->int_raw.val & intmask) &&
                   ((micros() - us) <= us_limit))
                ;
            if (0 != dev->sr.rx_fifo_cnt)
#else
            while (0 == dev->status_reg.rx_fifo_cnt &&
                   !(dev->int_raw.val & intmask) &&
                   ((micros() - us) <= us_limit))
                ;
            if (0 != dev->status_reg.rx_fifo_cnt)
#endif
            {
                *readdata++ = *fifo_addr;
            } else {
                i2c_stop();
                ESP_LOGW("I2C_Master", "readBytes error : read timeout");
                res    = false;
                _state = state_t::state_error;
                return res;
            }
        } while (--len);
    } while (length);

    return res;
}
//*/
bool I2C_Master::readWords(uint16_t *readdata, size_t length, bool last_nack,
                           int freq) {
    if (!length) {
        return true;
    }
    if (_state == state_t::state_error || _state == state_t::state_write) {
        return false;
    }
    auto i2c_dev = get_i2c_dev(_i2c_port);
    if (i2c_dev == nullptr) {
        return false;
    }
    if (freq) {
        i2c_wait(false);
        setFreq(freq);
    }

    _isr_semaphore     = xSemaphoreCreateBinary();
    _isr_recv_done_len = 0;
    _readword_inner(get_i2c_dev(_i2c_port), (uint8_t *)readdata, length,
                    last_nack);
    // 想定の8倍の時間(+10 msec)で応答が得られなければ通信失敗と見なして帰る
    auto result = xSemaphoreTake(_isr_semaphore,
                                 ((18 * 1000 * length) / (freq >> 3)) + 10);
    vSemaphoreDelete(_isr_semaphore);
    _isr_semaphore = nullptr;
    return _isr_result && (result == pdTRUE);
}
/*
bool I2C_Master::readWords(uint16_t *readdata, size_t length, bool last_nack) {
    auto i2c_dev = get_i2c_dev(_i2c_port);
    if (i2c_dev == nullptr) {
        return false;
    }
    if (_state == state_t::state_error) {
        return false;
    }
    if (_state == state_t::state_write) {
        return false;
    }
    if (!length) {
        return true;
    }
    bool res = true;

    static constexpr uint32_t intmask =
        I2C_ACK_ERR_INT_RAW_M | I2C_TIME_OUT_INT_RAW_M |
        I2C_END_DETECT_INT_RAW_M | I2C_ARBITRATION_LOST_INT_RAW_M;
    auto fifo_addr = getFifoAddr(_i2c_port);
    auto dev       = get_i2c_dev(_i2c_port);
    size_t len     = 0;

    uint32_t us_limit =
        (dev->scl_high_period.period + dev->scl_low_period.period + 16);
    do {
        len = ((length - 1) & 15) + 1;
        length -= len;
        res = i2c_wait(false);
        if (!res) {
            ESP_LOGW("I2C_Master", "readWords error : ack wait");
            break;
        }
        i2c_set_cmd(dev, 0, i2c_cmd_read, (len << 1) - 1);
        i2c_set_cmd(dev, 1, i2c_cmd_read, 1, length == 0 && last_nack);
        i2c_set_cmd(dev, 2, i2c_cmd_end, 0);

        dev->int_clr.val     = intmask;
        dev->ctr.trans_start = 1;
        if (length) {
            taskYIELD();
        }
        do {
            uint32_t us = micros();
#if defined(CONFIG_IDF_TARGET_ESP32C3)
            while (0 == dev->sr.rx_fifo_cnt && !(dev->int_raw.val & intmask) &&
                   ((micros() - us) <= us_limit))
                ;
            if (0 != dev->sr.rx_fifo_cnt)
#else
            while (2 > dev->status_reg.rx_fifo_cnt &&
                   !(dev->int_raw.val & intmask) &&
                   ((micros() - us) <= us_limit))
                ;
            if (1 < dev->status_reg.rx_fifo_cnt)
#endif
            {
                auto tmp    = *fifo_addr << 8;
                *readdata++ = tmp + *fifo_addr;
            } else {
                i2c_stop();
                ESP_LOGW("I2C_Master", "readWords error : read timeout");
                res    = false;
                _state = state_t::state_error;
            }
        } while (--len);
    } while (length);

    return res;
}
//*/
bool I2C_Master::transactionWrite(int addr, const uint8_t *writedata,
                                  uint8_t writelen, uint32_t freq) {
    return start(addr, false, freq) && writeBytes(writedata, writelen) &&
           stop();
}

bool I2C_Master::transactionRead(int addr, uint8_t *readdata, uint8_t readlen,
                                 uint32_t freq) {
    return start(addr, true, freq) && readBytes(readdata, readlen) && stop();
}

bool I2C_Master::transactionWriteRead(int addr, const uint8_t *writedata,
                                      uint8_t writelen, uint8_t *readdata,
                                      size_t readlen, uint32_t freq) {
    return start(addr, false, freq) && writeBytes(writedata, writelen) &&
           restart(addr, true, freq) && readBytes(readdata, readlen) && stop();
}
};  // namespace m5
