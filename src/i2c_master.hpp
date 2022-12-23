//! Copyright (c) M5Stack. All rights reserved.
//! Licensed under the MIT license.
//! See LICENSE file in the project root for full license information.

#pragma once

#include <driver/i2c.h>
#include <soc/i2c_struct.h>
#include <cstdint>

namespace m5 {
class I2C_Master {
   public:
    I2C_Master(void){};
    bool init(int i2c_port, int pin_sda, int pin_scl);
    bool release(void);

    bool setFreq(uint32_t freq);
    bool start(int i2c_addr, bool read, uint32_t freq);
    bool restart(int i2c_addr, bool read, uint32_t freq);
    bool stop(void);

    bool writeBytes(const uint8_t *data, size_t length);
    bool readBytes(uint8_t *readdata, size_t length, bool last_nack = false);
    bool writeWords(const uint16_t *data, size_t length);
    bool readWords(uint16_t *readdata, size_t length, bool last_nack, int freq);

    bool transactionWrite(int addr, const uint8_t *writedata, uint8_t writelen,
                          uint32_t freq);
    bool transactionRead(int addr, uint8_t *readdata, uint8_t readlen,
                         uint32_t freq);
    bool transactionWriteRead(int addr, const uint8_t *writedata,
                              uint8_t writelen, uint8_t *readdata,
                              size_t readlen, uint32_t freq);

   private:
    static void _isr_handler(void *arg);
    bool _readword_inner(i2c_dev_t *dev, uint8_t *data, size_t length,
                         bool last_nack);
    enum isr_mode_t {
        isr_nojob,
        isr_readword,
    };
    isr_mode_t _isr_mode            = isr_nojob;
    uint8_t *_isr_recv_buf          = nullptr;
    size_t _isr_recv_done_len       = 0;
    size_t _isr_recv_remain_len     = 0;
    bool _isr_result                = 0;
    xSemaphoreHandle _isr_semaphore = nullptr;

    gpio_num_t _pin_sda;
    gpio_num_t _pin_scl;
    uint32_t _freq;
    uint8_t _i2c_port;
    bool _wait_ack;

    enum state_t { state_disconnect, state_write, state_read, state_error };
    state_t _state;

    bool i2c_wait(bool flg_stop = false);
    void i2c_stop(void);

    void save_reg(void);
    void load_reg(void);

    uint32_t scl_high_period;
    uint32_t scl_low_period;
    uint32_t scl_start_hold;
    uint32_t scl_rstart_setup;
    uint32_t scl_stop_hold;
    uint32_t scl_stop_setup;
    uint32_t sda_hold;
    uint32_t sda_sample;
    uint32_t fifo_conf;
    uint32_t timeout;
    uint32_t filter_cfg;
    uint32_t scl_filter;
    uint32_t sda_filter;
};
};  // namespace m5
