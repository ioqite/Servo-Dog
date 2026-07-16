#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"

#include "esp_check.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

// 发送通道句柄
rmt_channel_handle_t tx_handle_t;
// 编码器句柄
rmt_encoder_handle_t ws2812b_encoder_handle_t;
// 通过组合原始编码器构建编码器
typedef struct
{
    rmt_encoder_t base;           // 基础类 "class" 声明了标准编码器接口
    rmt_encoder_t *copy_encoder;  // 使用拷贝编码器来编码前导码和结束码
    rmt_encoder_t *bytes_encoder; // 使用字节编码器来编码地址和命令数据
    int state;                    // 记录当前编码状态,即所处编码阶段
    rmt_symbol_word_t reset_code; // 结束位的时序
} rmt_ws2812b_encoder_t;
 
/*
    创建编码器发送回调函数
    rmt_encoder_t *encoder,            // RMT编码器对象（继承自父结构体）
    rmt_channel_handle_t channel,      // RMT硬件通道句柄
    const void *primary_data,          // 原始数据
    size_t data_size,                  // 原始数据大小（字节数）
    rmt_encode_state_t *ret_state      // 返回编码状态（完成/内存不足）
*/
static size_t rmt_encode_ws2812b_Send(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    /*
    __containerof宏的作用:
    通过结构的成员来访问这个结构的地址
    在这个函数中，传入参数encoder是rmt_led_strip_encoder_t结构体中的base成员
    __containerof宏通过encoder的地址，根据rmt_led_strip_encoder_t的内存排布找到rmt_led_strip_encoder_t* 的首地址
    */
    rmt_ws2812b_encoder_t *ws2812b_encoder = __containerof(encoder, rmt_ws2812b_encoder_t, base);

    rmt_encoder_handle_t bytes_encoder = ws2812b_encoder->bytes_encoder; // 取出字节编码器
    rmt_encoder_handle_t copy_encoder = ws2812b_encoder->copy_encoder;   // 取出拷贝编码器
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;               // 编码会话处于 重置 状态
    rmt_encode_state_t state = RMT_ENCODING_RESET;

    size_t encoded_symbols = 0;
    //有限状态机
    switch(ws2812b_encoder->state)
    {       // ws2812b_encoder->state是自定义的状态，这里只有两种值，0是发送RGB数据，1是发送复位码
    case 0: // send RGB data
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
        
        if (session_state & RMT_ENCODING_COMPLETE)
        {                               // 字节编码完成
            ws2812b_encoder->state = 1; // 当前编码会话完成后切换到下一个状态
        }

        if (session_state & RMT_ENCODING_MEM_FULL)
        { // 缓存不足，本次退出
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space for encoding artifacts
        }
    // fall-through
    case 1: // send reset code
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &ws2812b_encoder->reset_code,
                                                sizeof(ws2812b_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            ws2812b_encoder->state = RMT_ENCODING_RESET; // back to the initial encoding session
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space for encoding artifacts
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}
// 创建编码器卸载回调函数
static esp_err_t rmt_del_ws2812b_unload(rmt_encoder_t *encoder)
{
    rmt_ws2812b_encoder_t *ws2812b_encoder = __containerof(encoder, rmt_ws2812b_encoder_t, base);
    rmt_del_encoder(ws2812b_encoder->bytes_encoder); // 重置字节编码器
    rmt_del_encoder(ws2812b_encoder->copy_encoder);  // 重置拷贝编码器
    free(ws2812b_encoder);                           // 释放
    return ESP_OK;
}
// 创建编码器重置回调函数
static esp_err_t rmt_reset_ws2812b_reset(rmt_encoder_t *encoder)
{
    rmt_ws2812b_encoder_t *ws2812b_encoder = __containerof(encoder, rmt_ws2812b_encoder_t, base);
    rmt_encoder_reset(ws2812b_encoder->bytes_encoder); // 重置字节编码器
    rmt_encoder_reset(ws2812b_encoder->copy_encoder);  // 重置拷贝编码器
    ws2812b_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}
// 创建编码器
esp_err_t rmt_new_ws2812b_strip_encoder(rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;

    // 创建一个自定义的编码器结构体，用于控制发送编码的流程
    rmt_ws2812b_encoder_t *ws2812b_encoder = NULL;
    // 申请内存
    ws2812b_encoder = calloc(1, sizeof(rmt_ws2812b_encoder_t));
    ESP_GOTO_ON_FALSE(ws2812b_encoder, ESP_ERR_NO_MEM, err, "rmt_new_ws2812b_strip_encoder", "no mem for led strip encoder");
    // 挂载回调函数
    ws2812b_encoder->base.encode = rmt_encode_ws2812b_Send; // 这个函数会在rmt发送数据的时候被调用，我们可以在这个函数增加额外代码进行控制
    ws2812b_encoder->base.del = rmt_del_ws2812b_unload;     // 这个函数在卸载rmt时被调用
    ws2812b_encoder->base.reset = rmt_reset_ws2812b_reset;  // 这个函数在复位rmt时被调用

    rmt_bytes_encoder_config_t ws2812b_bytes =
        {
            // bit 0
            .bit0 =
                {
                    .duration0 = 0.3 * 10000000 / 1000000,
                    .level0 = 1,
                    .duration1 = 0.9 * 10000000 / 1000000,
                    .level1 = 0,
                    //.val = 0,
                },
            // bit 1
            .bit1 =
                {
                    .duration0 = 0.9 * 10000000 / 1000000,
                    .level0 = 1,
                    .duration1 = 0.3 * 10000000 / 1000000,
                    .level1 = 0,
                    //.val = 1,
                },
            // 高位优先
            .flags.msb_first = 1,
        };
    // 传入编码器配置，传入发送回调函数
    rmt_new_bytes_encoder(&ws2812b_bytes, &ws2812b_encoder->bytes_encoder); // 可以创建字节编码器
    // 新建一个拷贝编码器配置，拷贝编码器一般用于传输恒定的字符数据，比如说结束位

    // 可以创建拷贝编码器
    rmt_copy_encoder_config_t copy_encoder_config={};
    rmt_new_copy_encoder(&copy_encoder_config, &ws2812b_encoder->copy_encoder);

    // 设定结束位时序
    uint32_t reset_ticks = 10000000 / 1000000 * 50 / 2; // 分辨率/1M=每个ticks所需的us，然后乘以50就得出50us所需的ticks
    ws2812b_encoder->reset_code = (rmt_symbol_word_t){
        .level0 = 0,
        .duration0 = reset_ticks,
        .level1 = 0,
        .duration1 = reset_ticks,
    };

    // 返回编码器
    *ret_encoder = &ws2812b_encoder->base;
    return ESP_OK;
err:
    // 判断是否存在
    if (ws2812b_encoder)
    {
        if (ws2812b_encoder->bytes_encoder)
        {
            rmt_del_encoder(ws2812b_encoder->bytes_encoder); // 删除拷贝编码器
        }
        if (ws2812b_encoder->copy_encoder)
        {
            rmt_del_encoder(ws2812b_encoder->copy_encoder); // 删除字节编码器
        }
        free(ws2812b_encoder);
    }
    return ret;
}

void WS2812B_Init(uint8_t pin)
{
    rmt_tx_channel_config_t ws2812b_rmt =
        {
            .clk_src = RMT_CLK_SRC_DEFAULT, // 选择时钟源
            .gpio_num = pin,                // GPIO 编号
            .resolution_hz = 10000000,      // 1000hz 0.1us
            .mem_block_symbols = 64,        // 内存块大小
            .trans_queue_depth = 4,         // 设置后台等待处理的事务数量
        };
    // 创建发射通道
    ESP_ERROR_CHECK(rmt_new_tx_channel(&ws2812b_rmt,&tx_handle_t));
    // 创建自定义编码器
    ESP_ERROR_CHECK(rmt_new_ws2812b_strip_encoder(&ws2812b_encoder_handle_t));
    // 使能RMT通道
    ESP_ERROR_CHECK(rmt_enable(tx_handle_t));
}
void app_main(void)
{
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, //不循环发送
    };
    uint8_t rgb[126]={0x00,0x0f,0xff,0xff,0x0f,0xff,0x00,0x0ff,0xff,
                    0x00,0x0f,0xff,0xff,0x0f,0xff,0x00,0x0ff,0xff,
                    0x00,0x0f,0xff,0xff,0x0f,0xff,0x00,0x0ff,0xff,
                    0x00,0x0f,0xff,0xff,0x0f,0xff,0x00,0x0ff,0xff,
                    0x00,0x0f,0xff,0xff,0x0f,0xff,0x00,0x0ff,0xff,
                    0x00,0x0f,0xff,0xff,0x0f,0xff,0x00,0x0ff,0xff,
                    0x00,0x0f,0xff,0xff,0x0f,0xff,0x00,0x0ff,0xff, 
                    0x00,0x0f,0xff,0xff,0x0f,0xff,0x00,0x0ff,0xff,
                    0x00,0x0f,0xff,0xff,0x0f,0xff,0x00,0x0ff,0xff,
                    0x00,0x0f,0xff,0xff,0x0f,0xff,0x00,0x0ff,0xff,
                    0x00,0x0f,0xff,0xff,0x0f,0xff,0x00,0x0ff,0xff,
                    0x00,0x0f,0xff,0xff,0x0f,0xff,0x00,0x0ff,0xff,
                    0x00,0x0f,0xff,0xff,0x0f,0xff,0x00,0x0ff,0xff,
                    0x00,0x0f,0xff,0xff,0x0f,0xff,0x00,0x0ff,0xff,                    
    };
    WS2812B_Init(18);
    while (1)
    {
        rmt_transmit(tx_handle_t ,ws2812b_encoder_handle_t,rgb,126, &tx_config);
        vTaskDelay(100);
    }
}
