#pragma once

#include <cstdint>
#include <cstddef>

namespace m5 {
class I2C_Master;

class MLX90640_Class {
   public:
    // Amount of bit shift required for conversion to Celsius temperature.
    static constexpr int DATA_RATIO_SHIFT = 7;

    // Magnification required for conversion to Celsius temperature.
    static constexpr int DATA_RATIO_VALUE = 1 << DATA_RATIO_SHIFT;

    // Negative amount required for conversion to Celsius temperature.
    static constexpr int DATA_OFFSET = 64;

    MLX90640_Class(void) : _i2c{nullptr} {};

    static constexpr size_t PIXEL_ROWS       = 24;
    static constexpr size_t PIXEL_COLS       = 32;
    static constexpr size_t FRAME_DATA_BYTES = 834 * 2;
    static constexpr size_t TEMP_DATA_BYTES  = 32 * 24 * 2;
    static constexpr size_t DATA_ARRAY_LEN   = 16 * 24;

    enum refresh_rate_t {
        rate_0_5Hz,
        rate_1Hz,
        rate_2Hz,
        rate_4Hz,
        rate_8Hz,
        rate_16Hz,
        rate_32Hz,
        rate_64Hz,
    };

#pragma pack(push)
#pragma pack(1)
    struct temperature_info_t {
        uint16_t temp;
        uint8_t x;
        uint8_t y;
    };
    struct temp_data_t {
        uint8_t refresh_control;
        uint8_t subpage;
        uint16_t med_temp;
        uint16_t avg_temp;
        temperature_info_t diff_info;
        temperature_info_t min_info;
        temperature_info_t max_info;
        uint16_t data[16 * 24];
    };
#pragma pack(pop)

    bool init(I2C_Master* i2c);
    void setRate(refresh_rate_t rate);
    inline refresh_rate_t getRate(void) const {
        return _refresh_rate;
    }
    void update(void);

    bool writeReg(uint16_t reg, uint16_t value);
    bool writeReg(uint16_t reg, const uint16_t* data, size_t len);
    bool readReg(uint16_t reg, uint16_t* data, size_t len);

    /// read raw frame data from MLX90640
    /// framedata require size 834 * 2 Bytes
    bool readFrameData(uint16_t* framedata);

    /// calc by raw frame data
    /// tempdata require 32*24*2 Byte, and framedata require size 834 * 2 Bytes
    void calcTempData(const uint16_t* framedata, temp_data_t* tempdata,
                      const temp_data_t* prev_tempdata, uint32_t filter_level,
                      uint8_t monitor_width, uint8_t monitor_height);

    void calcTempData(const uint16_t* framedata, temp_data_t* tempdata,
                      float emissivity);

   private:
    I2C_Master* _i2c;
    refresh_rate_t _refresh_rate = (refresh_rate_t)-1;
    uint32_t _i2c_freq           = 800000;
    uint8_t _i2c_addr            = 0x33;
};
}  // namespace m5