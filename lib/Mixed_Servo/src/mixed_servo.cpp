#include "mixed_servo.hpp"

namespace m_servo {  // Mixed_Servo 命名空间

// ----- 舵机引脚 -----
uint8_t _servos_pin[TOTAL_SERVO_NUM]       = {4,  5, 11, 10,      14, 12,  8,  3};
int16_t _servo_correction[TOTAL_SERVO_NUM] = {3, -3,  2, 10,      -1,  0,  0,  0};
bool _servo_local[TOTAL_SERVO_NUM]         = {0,  1,  0,  1,       0,  1,  0,  1};

std::array<int16_t, TOTAL_SERVO_NUM> cur = {0,  0,  0,  0,       0,  0,  0,  0};

// ============= MCPWM 舵机 ==============
mcpwm_cmpr_handle_t __comparators[MCPWM_SERVO_NUM];
mcpwm_gen_handle_t  __generators[MCPWM_SERVO_NUM];

// ============== LEDC 舵机 ===============
// 无

// 原始角度 控制函数 (舵机编号 0-7，角度 0 ~ 180), 自动处理 MCPWM 和 LED 舵机
void set_angle_raw(uint8_t servo_idx, int16_t angle) {
    // 0-3 MCPWM 舵机
    if (servo_idx >= 0 && servo_idx < MCPWM_SERVO_NUM) {
        angle = constrain(angle, 0, 180);
        uint32_t pulse_us = map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
        
        // 更新对应通道的比较值
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(__comparators[servo_idx], pulse_us));
    }
    // 4-7 LEDC 舵机
    else if (servo_idx >= MCPWM_SERVO_NUM && servo_idx < TOTAL_SERVO_NUM) {
        angle = constrain(angle, 0, 180);
        uint32_t pulse_us = map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);

        // 将脉冲宽度转换为 13 位分辨率的占空比值
        // 公式：(脉冲宽度 / (2^分辨率 - 1)) * 总周期
        uint32_t duty = (pulse_us * (1<<LEDC_RESOLUTION_HZ)-1) / LEDC_PERIOD;

        // 输出 PWM
        ledcWrite(_servos_pin[servo_idx], duty);
    }
    else ESP_LOGE("Mixed_Servo", "Invalid servo index: %d", servo_idx);
}
// 角度控制函数 (舵机编号 0-7，角度 0 ~ 180)
void set_angle(uint8_t servo_idx, int16_t angle) {
    if (_servo_local[servo_idx]) {
		set_angle_raw(servo_idx, angle + _servo_correction[servo_idx]);
	} else {
		set_angle_raw(servo_idx, 180 - angle - _servo_correction[servo_idx]);
	}
    cur[servo_idx] = angle - 90;
}
// 角度控制函数 (舵机编号 0-7，角度 -90 ~ 90)
void set_angle_90(uint8_t servo_index, int16_t angle) {
	if (_servo_local[servo_index]) {
		set_angle_raw(servo_index, (angle + 90) + _servo_correction[servo_index]);
	} else {
		set_angle_raw(servo_index, 180 - (angle + 90) - _servo_correction[servo_index]);
	}
    cur[servo_index] = angle;
}
// 同时将多个舵机 设置到 目标角度(-90 ~ 90)
void set_angle_90_multi(std::array<int16_t, TOTAL_SERVO_NUM> targets) {
    for (uint8_t i = 0; i < TOTAL_SERVO_NUM; i++) {
		set_angle_90(i, targets[i]);
	}
}

// 角度渐变函数 (舵机编号 0-7，角度 0 ~ 180)
void to_angle(uint8_t servo_idx, float time_ms, float cur, float target) {
    if (target > cur) {
        float step = (target - cur) / time_ms;
        if (step == 0) step = 1;
		for (float agl = cur; agl <= target; agl += step) {
			set_angle(servo_idx, (int16_t)agl);
			delay(1);
		}
    } else {
        float step = (cur - target) / time_ms;
        if (step == 0) step = 1;
		for (float agl = cur; agl >= target; agl -= step) {
			set_angle(servo_idx, (int16_t)agl);
			delay(1);
		}
    }
}

void to_angle_90(uint8_t servo_idx, float time_ms, float cur, float target) {
    if (target > cur) {
        float step = (target - cur) / time_ms;
        if (step == 0) step = 1;
		for (float agl = cur; agl <= target; agl += step) {
			set_angle_90(servo_idx, (int16_t)agl);
			delay(1);
		}
    } else {
        float step = (cur - target) / time_ms;
        if (step == 0) step = 1;
		for (float agl = cur; agl >= target; agl -= step) {
			set_angle_90(servo_idx, (int16_t)agl);
			delay(1);
		}
    }
}

// 同时将多个舵机平滑地转到目标角度(相对90度偏移)
void to_angle_90_sync(std::array<int16_t, TOTAL_SERVO_NUM> target, float time_ms) {
	bool moving = true;
	while (moving) {
		moving = false;
		for (uint8_t i = 0; i < TOTAL_SERVO_NUM; i++) {
			if (cur[i] < target[i]) { cur[i]++; moving = true; }
			else if (cur[i] > target[i]) { cur[i]--; moving = true; }
			set_angle_90(i, cur[i]);
		}
		if (moving) delay(1);
    }
}

// 初始化 MCPWM 舵机
void __setup_mcpwm() {
    // -------- 1. 创建并配置共享 timer --------
    // Serial.println("[MCPWM] 创建 timer");
    mcpwm_timer_handle_t timer = NULL;
    const mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = MCPWM_TIMER_PERIOD,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // -------- 2. 创建 2 个 operator，并连接到同一个 timer --------
    // 每个 operator 最多带 2 个 generator，所以 4 路需要 2 个 operator
    // Serial.println("[MCPWM] 创建 operator");
    mcpwm_oper_handle_t operators[2];
    const mcpwm_operator_config_t operator_config = {
        .group_id = 0,  // operator must be in the same group as the timer
    };
    for (uint8_t i = 0; i < 2; i++) {
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[i]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], timer));
    }

    // -------- 3. 每个 operator 创建 2 个 comparator 和 2 个 generator --------
    // Serial.println("[MCPWM] 创建 comparator 和 generator");
    const mcpwm_comparator_config_t comparator_config = {
        .flags = { .update_cmp_on_tez = true },
    };

    for (uint8_t i = 0; i < MCPWM_SERVO_NUM; i++) {
        uint8_t op_idx = i / 2;  // operator 0 -> servo 0,1 ; operator 1 -> servo 2,3

        ESP_ERROR_CHECK(mcpwm_new_comparator(operators[op_idx], &comparator_config, &__comparators[i]));

        const mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = _servos_pin[i],
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(operators[op_idx], &generator_config, &__generators[i]));

        // 设置 初始角度
        set_angle_raw(i, DEFAULT_SERVO_ANGLE);

        // 计数器清零（empty）时输出高电平
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            __generators[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                         MCPWM_TIMER_EVENT_EMPTY,
                                         MCPWM_GEN_ACTION_HIGH)));
        // 比较匹配时输出低电平（产生脉宽）
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            __generators[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                           __comparators[i],
                                           MCPWM_GEN_ACTION_LOW)));
    }

    // -------- 4. 启动 timer --------
    // Serial.println("[MCPWM] 启动 timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}
// 初始化 LEDC 舵机
void __setup_ledc() {
    for (uint8_t i = 0; i < LEDC_SERVO_NUM; i++) {
        ledcAttach(_servos_pin[i+MCPWM_SERVO_NUM], LEDC_FREQ_HZ, LEDC_RESOLUTION_HZ);
         // 设置 初始角度
        set_angle(i+MCPWM_SERVO_NUM, DEFAULT_SERVO_ANGLE);
    }
}

// 初始化舵机
void setup_servos(uint8_t pin[TOTAL_SERVO_NUM], int16_t correction[TOTAL_SERVO_NUM]) {
    memcpy(_servos_pin, pin, sizeof(uint8_t) * TOTAL_SERVO_NUM);
    memcpy(_servo_correction, correction, sizeof(int16_t) * TOTAL_SERVO_NUM);
    __setup_mcpwm();
    __setup_ledc();
}

} // namespace m_servo

