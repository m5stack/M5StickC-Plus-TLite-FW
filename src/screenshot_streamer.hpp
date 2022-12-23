//! Copyright (c) M5Stack. All rights reserved.
//! Licensed under the MIT license.
//! See LICENSE file in the project root for full license information.

#include <stdint.h>
#include <stddef.h>

#include <M5GFX.h>
#include <WiFiClient.h>
#include "jpg/jpge.h"

class screenshot_streamer_t : public jpge::output_stream {
    QueueHandle_t _queue_canvas;
    QueueHandle_t _queue_client;
    QueueHandle_t _queue_bufdata;

    WiFiClient* _wifi_client;

    struct queue_ss_t {
        M5Canvas* canvas;
        uint16_t y;
    };

    struct queue_bufdata_t {
        const uint8_t* bufdata;
        size_t len;
        bool y0;
    };
    queue_bufdata_t _q_bufdata;

    uint16_t _y      = 0;
    uint16_t _width  = 0;
    uint16_t _height = 0;
    bool _is_requested;

    jpge::jpeg_encoder _jpeg_enc;

   public:
    screenshot_streamer_t(void);

    static void streamTask(void*);

    bool isRequested(void) const {
        return _is_requested;
    };
    void requestScreenShot(WiFiClient* client);

    // 新設
    bool initCapture(uint16_t width, uint16_t height);
    bool addQueue(M5Canvas* canvas, uint16_t y);

    enum process_result_t {
        pr_nothing,
        pr_error,
        pr_progress,
        pr_complete,
    };

    process_result_t processCapture(void);

    uint get_size(void) const override {
        return 0;
    }
    bool put_buf(const void* Pbuf, int len) override;
};

extern screenshot_streamer_t screenshot_holder;
