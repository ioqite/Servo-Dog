#include <Arduino.h>
#include "driver/mcpwm_prelude.h"

// 通用参数
#define TOTAL_SERVO_NUM       8  // 总舵机数
#define DEFAULT_SERVO_ANGLE      90 // 默认角度，90度
#define SERVO_MIN_PULSE_US          500      // 最小脉冲宽度，微秒
#define SERVO_MAX_PULSE_US          2500     // 最大脉冲宽度，微秒
// MCPWM 舵机
#define MCPWM_SERVO_NUM      4  // MCPWM 舵机数
#define MCPWM_TIMER_RESOLUTION_HZ   1000000  // 分辨率: 1MHz, 1微秒 / tick
#define MCPWM_TIMER_PERIOD          20000    // 周期:   20000 ticks, 20ms

int angles[MCPWM_SERVO_NUM] = {  90,  90, 90,  90};
int steps [MCPWM_SERVO_NUM] = {  -2,   2,  -2,   2};

static const int _servos_pin[MCPWM_SERVO_NUM] = { 4, 5, 11, 10 };

mcpwm_cmpr_handle_t comparators[MCPWM_SERVO_NUM];
mcpwm_gen_handle_t  generators[MCPWM_SERVO_NUM];

static inline uint32_t example_angle_to_compare(int angle) {
    return angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 180 + SERVO_MIN_PULSE_US;
}

void setup() {
    Serial.begin(115200);

    // -------- 1. 创建并配置共享 timer --------
    Serial.println("Create MCPWM timer");
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
    mcpwm_oper_handle_t operators[2];
    const mcpwm_operator_config_t operator_config = {
        .group_id = 0,  // operator must be in the same group as the timer
    };
    for (int i = 0; i < 2; i++) {
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[i]));
        Serial.printf("Connect timer and operator[%d]\n", i);
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], timer));
    }

    // -------- 3. 每个 operator 创建 2 个 comparator 和 2 个 generator --------
    Serial.println("Create comparator and generator from the operator");
    

    const mcpwm_comparator_config_t comparator_config = {
        .flags = { .update_cmp_on_tez = true },
    };

    for (int i = 0; i < MCPWM_SERVO_NUM; i++) {
        int op_idx = i / 2;  // operator 0 -> servo 0,1 ; operator 1 -> servo 2,3

        ESP_ERROR_CHECK(mcpwm_new_comparator(operators[op_idx], &comparator_config, &comparators[i]));

        const mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = _servos_pin[i],
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(operators[op_idx], &generator_config, &generators[i]));

        // 初始角度设为 90（中位）
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[i], example_angle_to_compare(90)));

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

void loop() {
    // -------- 4 路舵机独立运动循环 --------
    for (int i = 0; i < MCPWM_SERVO_NUM; i++) {
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(
            comparators[i], example_angle_to_compare(angles[i])));

        if ((angles[i] + steps[i]) > 130 || (angles[i] + steps[i]) < 40) {
            steps[i] *= -1;
        }
        angles[i] += steps[i];
    }
    // 200ms/60degree @5V，这里给 500ms 余量
    vTaskDelay(pdMS_TO_TICKS(500));
}
