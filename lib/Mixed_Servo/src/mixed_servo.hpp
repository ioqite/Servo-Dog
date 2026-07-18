#pragma once
#include <Arduino.h>
#include <array>
#include <string.h>
#include "driver/mcpwm_prelude.h"

namespace m_servo {  // Mixed_Servo 命名空间

// =============== 通用参数 ===============
#define TOTAL_SERVO_NUM             8   // 总舵机数
#define DEFAULT_SERVO_ANGLE         90  // 默认角度，90度
#define SERVO_MIN_PULSE_US          500    // 最小脉冲宽度，微秒
#define SERVO_MAX_PULSE_US          2500   // 最大脉冲宽度，微秒

// ============= MCPWM 舵机 ==============
#define MCPWM_SERVO_NUM            4        // MCPWM 舵机数
#define MCPWM_TIMER_RESOLUTION_HZ  1000000  // 分辨率: 1MHz, 1微秒 / tick
#define MCPWM_TIMER_PERIOD         20000    // 周期:   20000 ticks, 20ms

// ============== LEDC 舵机 ===============
#define LEDC_SERVO_NUM             4   // LEDC 舵机数
#define LEDC_RESOLUTION_HZ        13   // 分辨率: 13位
#define LEDC_FREQ_HZ              50   // 频率:   50Hz
#define LEDC_PERIOD            20000   // 周期:   20000 微秒, 20ms


// 原始角度 控制函数 (舵机编号 0-7，角度 0 ~ 180), 自动处理 MCPWM 和 LED 舵机
void set_angle_raw(uint8_t servo_idx, int16_t angle) ;

// 角度控制函数 (舵机编号 0-7，角度 0 ~ 180)
void set_angle(uint8_t servo_idx, int16_t angle);

// 角度控制函数 (舵机编号 0-7，角度 -90 ~ 90)
void set_angle_90(uint8_t servo_index, int16_t angle);

// 同时将多个舵机 设置到 目标角度(-90 ~ 90)
void set_angle_90_multi(std::array<int16_t, TOTAL_SERVO_NUM> targets);

// 角度渐变函数 (舵机编号 0-7，角度 0 ~ 180)
void to_angle(uint8_t servo_idx, float time_ms, float cur, float target);

void to_angle_90(uint8_t servo_idx, float time_ms, float cur, float target);

// 同时将多个舵机平滑地转到目标角度(相对90度偏移)
void to_angle_90_sync(std::array<int16_t, TOTAL_SERVO_NUM> target, float time_ms);

// 初始化 MCPWM 舵机
void __setup_mcpwm();

// 初始化 LEDC 舵机
void __setup_ledc();

// 初始化舵机
void setup_servos(uint8_t pin[TOTAL_SERVO_NUM], int16_t correction[TOTAL_SERVO_NUM]);

} // namespace m_servo

