#pragma once
#include <Arduino.h>
#include "esp_idf_version.h"

// 兼容 v2.x 和 v3.x 的 RMT 头文件
#if ESP_IDF_VERSION_MAJOR < 5
    #include "driver/rmt.h"
#else
    #include "driver/rmt_tx.h"
    #include "driver/rmt_encoder.h"
#endif

class WS2812 {
public:
    /**
     * @brief 构造函数
     * @param pin 连接 WS2812 的数据引脚
     * @param num LED 灯珠数量
     * @param channel RMT 通道 (仅 v2.x 有效, v3.x 自动分配)
     */
    WS2812(int pin, int num, int channel = 0)
        : _pin(pin), _num(num), _channel(channel), _buf(nullptr), _items(nullptr), _initialized(false) {
#if ESP_IDF_VERSION_MAJOR >= 5
        _tx_chan = nullptr;
        _encoder = nullptr;
#endif
    }

    ~WS2812() {
        if (_buf) free(_buf);
        if (_items) free(_items);
#if ESP_IDF_VERSION_MAJOR < 5
        if (_initialized) rmt_driver_uninstall((rmt_channel_t)_channel);
#else
        if (_encoder) rmt_del_encoder(_encoder);
        if (_tx_chan) rmt_disable(_tx_chan);
#endif
    }

    // 初始化 RMT 外设和内存
    void begin() {
        if (_num <= 0) return;
        // 分配颜色缓冲区 (GRB格式)
        _buf = (uint8_t*)calloc(_num, 3);
        // 分配 RMT 信号项缓冲区 (每个灯珠24bit，每个bit对应一个RMT item)
        _items = calloc(_num * 24, sizeof(uint32_t)); 

#if ESP_IDF_VERSION_MAJOR < 5
        // --- ESP32 Arduino v2.x (IDF v4.x) ---
        rmt_config_t config = RMT_DEFAULT_CONFIG_TX((gpio_num_t)_pin, (rmt_channel_t)_channel);
        config.clk_div = 2; // 80MHz / 2 = 40MHz (1 tick = 25ns)
        rmt_config(&config);
        rmt_driver_install(config.channel, 0, 0);
#else
        // --- ESP32 Arduino v3.x (IDF v5.x) ---
        rmt_tx_channel_config_t tx_config = {};
        tx_config.gpio_num = (gpio_num_t)_pin;
        tx_config.clk_src = RMT_CLK_SRC_DEFAULT;
        tx_config.resolution_hz = 10000000; // 10MHz (1 tick = 100ns)
        tx_config.mem_block_symbols = 64;
        tx_config.trans_queue_depth = 4;
        rmt_new_tx_channel(&tx_config, &_tx_chan);
        rmt_enable(_tx_chan);

        rmt_copy_encoder_config_t enc_config = {};
        rmt_new_copy_encoder(&enc_config, &_encoder);
#endif
        _initialized = true;
    }

    // 设置某个灯珠颜色
    void setPixel(int idx, uint8_t r, uint8_t g, uint8_t b) {
        if (!_buf || idx < 0 || idx >= _num) return;
        _buf[idx * 3] = g;     // WS2812硬件内部为GRB顺序
        _buf[idx * 3 + 1] = r;
        _buf[idx * 3 + 2] = b;
    }

    // 清除所有灯珠
    void clear() {
        if (!_buf) return;
        memset(_buf, 0, _num * 3);
        show();
    }

    // 将颜色缓冲推送到硬件显示
    void show() {
        if (!_buf || !_items || !_initialized) return;

#if ESP_IDF_VERSION_MAJOR < 5
        // --- v2.x: 使用 rmt_item32_t ---
        rmt_item32_t* items = (rmt_item32_t*)_items;
        for (int i = 0; i < _num; i++) {
            uint32_t color = (_buf[i * 3] << 16) | (_buf[i * 3 + 1] << 8) | _buf[i * 3 + 2];
            for (int j = 23; j >= 0; j--) {
                bool bit = (color >> j) & 1;
                int idx = i * 24 + (23 - j);
                if (bit) {
                    items[idx].level0 = 1; items[idx].duration0 = 32; // T1H: 800ns
                    items[idx].level1 = 0; items[idx].duration1 = 18; // T1L: 450ns
                } else {
                    items[idx].level0 = 1; items[idx].duration0 = 16; // T0H: 400ns
                    items[idx].level1 = 0; items[idx].duration1 = 34; // T0L: 850ns
                }
            }
        }
        rmt_write_items((rmt_channel_t)_channel, items, _num * 24, false);
        rmt_wait_tx_done((rmt_channel_t)_channel, pdMS_TO_TICKS(100));
#else
        // --- v3.x: 使用 rmt_symbol_word_t ---
        rmt_symbol_word_t* items = (rmt_symbol_word_t*)_items;
        for (int i = 0; i < _num; i++) {
            uint32_t color = (_buf[i * 3] << 16) | (_buf[i * 3 + 1] << 8) | _buf[i * 3 + 2];
            for (int j = 23; j >= 0; j--) {
                bool bit = (color >> j) & 1;
                int idx = i * 24 + (23 - j);
                if (bit) {
                    items[idx].level0 = 1; items[idx].duration0 = 8; // T1H: 800ns
                    items[idx].level1 = 0; items[idx].duration1 = 4; // T1L: 400ns
                } else {
                    items[idx].level0 = 1; items[idx].duration0 = 4; // T0H: 400ns
                    items[idx].level1 = 0; items[idx].duration1 = 8; // T0L: 800ns
                }
            }
        }
        rmt_transmit_config_t tx_config = {};
        tx_config.loop_count = 0;
        rmt_transmit(_tx_chan, _encoder, items, _num * 24 * sizeof(rmt_symbol_word_t), &tx_config);
        rmt_tx_wait_all_done(_tx_chan, 100);
#endif
    }

private:
    int _pin;
    int _num;
    int _channel;
    bool _initialized;
    uint8_t* _buf;
    void* _items; // 使用 void* 跨版本隐藏 rmt_item32_t / rmt_symbol_word_t 类型差异

#if ESP_IDF_VERSION_MAJOR >= 5
    rmt_channel_handle_t _tx_chan;
    rmt_encoder_handle_t _encoder;
#endif
};

