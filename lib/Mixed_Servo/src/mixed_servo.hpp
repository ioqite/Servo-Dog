#pragma once

#include <Arduino.h>
#include "driver/mcpwm_prelude.h"

namespace m_servo { // Mixed_Servo 命名空间

// 通用参数
#define TOTAL_SERVO_NUM       8  // 总舵机数
#define DEFAULT_SERVO_ANGLE      90 // 默认角度，90度
#define SERVO_MIN_PULSE_US          500      // 最小脉冲宽度，微秒
#define SERVO_MAX_PULSE_US          2500     // 最大脉冲宽度，微秒
// MCPWM 舵机
#define MCPWM_SERVO_NUM      4  // MCPWM 舵机数
#define MCPWM_TIMER_RESOLUTION_HZ   1000000  // 分辨率: 1MHz, 1微秒 / tick
#define MCPWM_TIMER_PERIOD          20000    // 周期:   20000 ticks, 20ms
mcpwm_cmpr_handle_t comparators[MCPWM_SERVO_NUM];
mcpwm_gen_handle_t  generators[MCPWM_SERVO_NUM];
// LEDC 舵机
#define LEDC_SERVO_NUM      4  // LEDC 舵机数
#define LEDC_RESOLUTION_HZ          12      // 分辨率: 12位
#define LEDC_FREQ_HZ                5000    // 频率:   5kHZ


// //                                    左前 右前 左后 右后
// uint8_t _servos_pin[TOTAL_SERVO_NUM]       = {4,  5,  7,  6};
// // uint8_t _servos_pin[TOTAL_SERVO_NUM]       = {10,  8,  3,  2};
// int16_t _servo_correction[TOTAL_SERVO_NUM] = {2, -1,  2,  9};
// // int16_t _servo_correction[TOTAL_SERVO_NUM] = {-1, 0,  0,  0};
// bool _servo_local[TOTAL_SERVO_NUM]         = {0,  1,  0,  1};

//                                         左前 右前 左后 右后
uint8_t _servos_pin[TOTAL_SERVO_NUM]       = {4,  5, 10, 11,      14, 12,  8,  3};
int16_t _servo_correction[TOTAL_SERVO_NUM] = {2, -1,  2,  9,      -1,  0,  0,  0};
bool _servo_local[TOTAL_SERVO_NUM]         = {0,  1,  0,  1,       0,  1,  0,  1};



// 原始角度 控制函数 (舵机编号 0-7，角度 0 ~ 180), 自动处理 MCPWM 和 LED 舵机
void set_angle_raw(int8_t servo_idx, int16_t angle) {
    // 0-3 MCPWM 舵机
    if (servo_idx >= 0 && servo_idx < MCPWM_SERVO_NUM) {
        Serial.print("set_angle_raw: mcpwm: ");
        Serial.print(servo_idx);
        Serial.print(", ");
        Serial.println(angle);
        angle = constrain(angle, 0, 180);
        uint32_t pulse_us = map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
        
        // 更新对应通道的比较值
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[servo_idx], pulse_us));
    }
    // 4-7 LEDC 舵机
    else if (servo_idx >= MCPWM_SERVO_NUM && servo_idx < TOTAL_SERVO_NUM) {
        Serial.print("set_angle_raw: ledc: ");
        Serial.print(servo_idx);
        Serial.print(", ");
        Serial.println(angle);
        angle = constrain(angle, 0, 180);
        uint32_t pulse_us = map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
        
        ledcWrite(_servos_pin[servo_idx], pulse_us);
    }
    else ESP_LOGE("Mixed_Servo", "Invalid servo index: %d", servo_idx);
}

// 角度控制函数 (舵机编号 0-7，角度 0 ~ 180)
void set_angle(int8_t servo_idx, int16_t angle) {
    if (_servo_local[servo_idx]) {
		set_angle_raw(servo_idx, angle + _servo_correction[servo_idx]);
	} else {
		set_angle_raw(servo_idx, 180 - angle + _servo_correction[servo_idx]);
	}
}
// 角度控制函数 (舵机编号 0-7，角度 -90 ~ 90)
void set_angle_90(int8_t servo_index, int16_t angle) {
	if (_servo_local[servo_index]) {
		set_angle(servo_index, (angle + 90) + _servo_correction[servo_index]);
	} else {
		set_angle(servo_index, 180 - (angle + 90) + _servo_correction[servo_index]);
	}
}

// 同时将多个舵机 设置到 目标角度(-90 ~ 90)
void set_angle_90_multi(std::array<int16_t, TOTAL_SERVO_NUM> targets) {
    for (int8_t i = 0; i < TOTAL_SERVO_NUM; i++) {
		set_angle_90(i, targets[i]);
	}
}

// 初始化 MCPWM 舵机
void __setup_mcpwm() {
    // -------- 1. 创建并配置共享 timer --------
    Serial.println("Create MCPWM timer");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = MCPWM_TIMER_PERIOD,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // -------- 2. 创建 2 个 operator，并连接到同一个 timer --------
    // 每个 operator 最多带 2 个 generator，所以 4 路需要 2 个 operator
    mcpwm_oper_handle_t operators[2];
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,  // operator must be in the same group as the timer
    };
    for (int i = 0; i < 2; i++) {
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[i]));
        Serial.printf("Connect timer and operator[%d]\n", i);
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], timer));
    }

    // -------- 3. 每个 operator 创建 2 个 comparator 和 2 个 generator --------
    Serial.println("Create comparator and generator from the operator");
    

    mcpwm_comparator_config_t comparator_config = {
        .flags = {
            .update_cmp_on_tez = true
        },
    };

    for (int i = 0; i < MCPWM_SERVO_NUM; i++) {
        int op_idx = i / 2;  // operator 0 -> servo 0,1 ; operator 1 -> servo 2,3

        ESP_ERROR_CHECK(mcpwm_new_comparator(operators[op_idx], &comparator_config, &comparators[i]));

        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = _servos_pin[i],
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(operators[op_idx], &generator_config, &generators[i]));

        // 初始角度
        set_angle(i, DEFAULT_SERVO_ANGLE);

        // 计数器清零（empty）时输出高电平
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            generators[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                         MCPWM_TIMER_EVENT_EMPTY,
                                         MCPWM_GEN_ACTION_HIGH)));
        // 比较匹配时输出低电平（产生脉宽）
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            generators[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                           comparators[i],
                                           MCPWM_GEN_ACTION_LOW)));
    }

    // -------- 4. 启动 timer --------
    Serial.println("Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

// 初始化 LEDC 舵机
void __setup_ledc() {
    for (int i = 0; i < LEDC_SERVO_NUM; i++) {
        ledcAttach(_servos_pin[i], LEDC_FREQ_HZ, LEDC_RESOLUTION_HZ);
         // 初始角度
        set_angle(i+4, DEFAULT_SERVO_ANGLE);
    }
}

// 初始化舵机
void setup_servos(uint8_t pin[TOTAL_SERVO_NUM], int16_t correction[TOTAL_SERVO_NUM]) {
    memcpy(_servos_pin, pin, sizeof(uint8_t) * TOTAL_SERVO_NUM);
    memcpy(_servo_correction, correction, sizeof(int16_t) * TOTAL_SERVO_NUM);
    for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
        Serial.print(_servos_pin[i]);Serial.print("\t");
        Serial.print(_servo_correction[i]);Serial.println();
    }
    __setup_mcpwm();
    __setup_ledc();
}

} // namespace m_servo
