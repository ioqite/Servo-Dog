#pragma once
#include <Arduino.h>
#include "esp_idf_version.h"
#include <stdlib.h>
#include <string.h>

#if ESP_IDF_VERSION_MAJOR < 5
    // --- ESP32 Arduino v2.x (IDF v4.x) ---
    #include "driver/rmt.h"
#else
    // --- ESP32 Arduino v3.x (IDF v5.x) ---
    #include "driver/rmt_tx.h"
    #include "driver/rmt_encoder.h"
    #include "esp_err.h"

    #ifndef __containerof
    #define __containerof(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))
    #endif

    // 自定义编码器结构体
    typedef struct {
        rmt_encoder_t base;           // 基础编码器接口
        rmt_encoder_t *bytes_encoder; // 字节编码器，用于处理RGB数据
        rmt_encoder_t *copy_encoder;  // 拷贝编码器，用于处理复位码
        int state;                    // 编码状态机
        rmt_symbol_word_t reset_code; // 复位时序
    } rmt_ws2812_encoder_t;

    static size_t rmt_encode_ws2812(rmt_encoder_t *encoder, rmt_channel_handle_t channel, 
                                    const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state) {
        rmt_ws2812_encoder_t *ws2812_encoder = __containerof(encoder, rmt_ws2812_encoder_t, base);
        rmt_encoder_handle_t bytes_encoder = ws2812_encoder->bytes_encoder;
        rmt_encoder_handle_t copy_encoder = ws2812_encoder->copy_encoder;
        rmt_encode_state_t session_state = RMT_ENCODING_RESET;
        rmt_encode_state_t state = RMT_ENCODING_RESET;
        size_t encoded_symbols = 0;

        switch (ws2812_encoder->state) {
        case 0: // 发送 RGB 数据
            encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                ws2812_encoder->state = 1; // 切换到发送复位码状态
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                state = (rmt_encode_state_t)(state | RMT_ENCODING_MEM_FULL);
                goto out;
            }
        // fall-through
        case 1: // 发送复位码
            encoded_symbols += copy_encoder->encode(copy_encoder, channel, &ws2812_encoder->reset_code,
                                                    sizeof(ws2812_encoder->reset_code), &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                ws2812_encoder->state = RMT_ENCODING_RESET; // 回到初始状态
                state = (rmt_encode_state_t)(state | RMT_ENCODING_COMPLETE);
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                state = (rmt_encode_state_t)(state | RMT_ENCODING_MEM_FULL);
                goto out;
            }
        }
    out:
        *ret_state = state;
        return encoded_symbols;
    }

    static esp_err_t rmt_del_ws2812(rmt_encoder_t *encoder) {
        rmt_ws2812_encoder_t *ws2812_encoder = __containerof(encoder, rmt_ws2812_encoder_t, base);
        rmt_del_encoder(ws2812_encoder->bytes_encoder);
        rmt_del_encoder(ws2812_encoder->copy_encoder);
        free(ws2812_encoder);
        return ESP_OK;
    }

    static esp_err_t rmt_reset_ws2812(rmt_encoder_t *encoder) {
        rmt_ws2812_encoder_t *ws2812_encoder = __containerof(encoder, rmt_ws2812_encoder_t, base);
        rmt_encoder_reset(ws2812_encoder->bytes_encoder);
        rmt_encoder_reset(ws2812_encoder->copy_encoder);
        ws2812_encoder->state = RMT_ENCODING_RESET;
        return ESP_OK;
    }

    // 创建自定义编码器 (规避 C++ 联合体/结构体严格初始化的语法限制)
    static esp_err_t rmt_new_ws2812_encoder(rmt_encoder_handle_t *ret_encoder) {
        rmt_ws2812_encoder_t *ws2812_encoder = (rmt_ws2812_encoder_t *)calloc(1, sizeof(rmt_ws2812_encoder_t));
        if (!ws2812_encoder) return ESP_ERR_NO_MEM;
        
        ws2812_encoder->base.encode = rmt_encode_ws2812;
        ws2812_encoder->base.del = rmt_del_ws2812;
        ws2812_encoder->base.reset = rmt_reset_ws2812;

        // 配置字节编码器 (10MHz 下: 1 tick = 100ns)
        rmt_bytes_encoder_config_t bytes_config = {};
        bytes_config.bit0.duration0 = 3;  // T0H: 300ns
        bytes_config.bit0.level0 = 1;
        bytes_config.bit0.duration1 = 9;  // T0L: 900ns
        bytes_config.bit0.level1 = 0;
        
        bytes_config.bit1.duration0 = 9;  // T1H: 900ns
        bytes_config.bit1.level0 = 1;
        bytes_config.bit1.duration1 = 3;  // T1L: 300ns
        bytes_config.bit1.level1 = 0;
        
        bytes_config.flags.msb_first = 1; // WS2812 要求高位先行
        rmt_new_bytes_encoder(&bytes_config, &ws2812_encoder->bytes_encoder);
        
        // 配置拷贝编码器
        rmt_copy_encoder_config_t copy_config = {};
        rmt_new_copy_encoder(&copy_config, &ws2812_encoder->copy_encoder);

        // 设定结束位（复位码 > 50us）
        ws2812_encoder->reset_code.duration0 = 500; // 50us
        ws2812_encoder->reset_code.level0 = 0;
        ws2812_encoder->reset_code.duration1 = 500; // 50us
        ws2812_encoder->reset_code.level1 = 0;

        *ret_encoder = &ws2812_encoder->base;
        return ESP_OK;
    }
#endif

class WS2812 {
public:
    WS2812(int pin, int num, int channel = 0)
        : _pin(pin), _num(num), _channel(channel), _initialized(false), _buf(nullptr) {
#if ESP_IDF_VERSION_MAJOR >= 5
        _tx_chan = nullptr;
        _encoder = nullptr;
#else
        _items = nullptr;
#endif
    }

    ~WS2812() {
        if (_buf) free(_buf);
#if ESP_IDF_VERSION_MAJOR < 5
        if (_items) free(_items);
        if (_initialized) rmt_driver_uninstall((rmt_channel_t)_channel);
#else
        if (_encoder) rmt_del_encoder(_encoder);
        if (_tx_chan) rmt_disable(_tx_chan);
#endif
    }

    void begin() {
        if (_num <= 0) return;
        _buf = (uint8_t*)calloc(_num, 3); // GRB 缓冲区

#if ESP_IDF_VERSION_MAJOR < 5
        // --- v2.x 初始化 ---
        rmt_config_t config = RMT_DEFAULT_CONFIG_TX((gpio_num_t)_pin, (rmt_channel_t)_channel);
        config.clk_div = 2; // 80MHz / 2 = 40MHz (1 tick = 25ns)
        rmt_config(&config);
        rmt_driver_install(config.channel, 0, 0);
        _items = (rmt_item32_t*)calloc(_num * 24, sizeof(rmt_item32_t));
#else
        // --- v3.x 初始化 ---
        rmt_tx_channel_config_t tx_config = {};
        tx_config.gpio_num = (gpio_num_t)_pin;
        tx_config.clk_src = RMT_CLK_SRC_DEFAULT;
        tx_config.resolution_hz = 10000000; // 10MHz (1 tick = 100ns)
        tx_config.mem_block_symbols = 64;
        tx_config.trans_queue_depth = 4;
        rmt_new_tx_channel(&tx_config, &_tx_chan);
        rmt_new_ws2812_encoder(&_encoder);
        rmt_enable(_tx_chan);
#endif
        _initialized = true;
        clear();
    }

    void setBrightness(uint8_t brightness) {
        _brightness = brightness / 255;
    }

    void setPixelColor(int idx, uint8_t r, uint8_t g, uint8_t b) {
        if (!_buf || idx < 0 || idx >= _num) return;
        _buf[idx * 3] = g * _brightness;     // WS2812 硬件为 GRB 顺序
        _buf[idx * 3 + 1] = r * _brightness;
        _buf[idx * 3 + 2] = b * _brightness;
    }

    void clear() {
        if (!_buf) return;
        memset(_buf, 0, _num * 3);
        show();
    }

    void show() {
        if (!_buf || !_initialized) return;

#if ESP_IDF_VERSION_MAJOR < 5
        // --- v2.x: 手动预计算并发送 ---
        for (int i = 0; i < _num; i++) {
            uint32_t color = (_buf[i * 3] << 16) | (_buf[i * 3 + 1] << 8) | _buf[i * 3 + 2];
            for (int j = 23; j >= 0; j--) {
                bool bit = (color >> j) & 1;
                int idx = i * 24 + (23 - j);
                if (bit) {
                    _items[idx].level0 = 1; _items[idx].duration0 = 32; // T1H: 800ns
                    _items[idx].level1 = 0; _items[idx].duration1 = 18; // T1L: 450ns
                } else {
                    _items[idx].level0 = 1; _items[idx].duration0 = 16; // T0H: 400ns
                    _items[idx].level1 = 0; _items[idx].duration1 = 34; // T0L: 850ns
                }
            }
        }
        rmt_write_items((rmt_channel_t)_channel, _items, _num * 24, false);
        rmt_wait_tx_done((rmt_channel_t)_channel, pdMS_TO_TICKS(100));
#else
        // --- v3.x: 利用自定义编码器直接发送原始 RGB 数组 ---
        rmt_transmit_config_t tx_config = {};
        tx_config.loop_count = 0;
        rmt_transmit(_tx_chan, _encoder, _buf, _num * 3, &tx_config);
        rmt_tx_wait_all_done(_tx_chan, 100);
#endif
    }

private:
    int _pin;
    int _num;
    int _channel;
    bool _initialized;
    uint8_t* _buf;
    double _brightness;

#if ESP_IDF_VERSION_MAJOR < 5
    rmt_item32_t* _items;
#else
    rmt_channel_handle_t _tx_chan;
    rmt_encoder_handle_t _encoder;
#endif
};

