
// // #include <Arduino.h>
// // #include "driver/mcpwm.h"
// // #include "soc/mcpwm_reg.h"
// // #include "soc/mcpwm_struct.h"
// // // #include "driver/mcpwm_prelude.h"

// // #define MOTO_GPIO1  4
// // #define MOTO_GPIO2  5
// // #define MOTO_GPIO3  11
// // #define MOTO_GPIO4  10
// // #define MOTO_TIMER MCPWM_TIMER_0
// // // 步骤一：选择mcpwm_unit
// // #define MOTOR_MCPWM_UNIT MCPWM_UNIT_0

// // // 初始化需要的四个gpio口

// // void setup(){
// // // 步骤二：用选定的mcpwm_unit来初始化gpio口
// // 	mcpwm_gpio_init(MOTOR_MCPWM_UNIT, MCPWM0A, MOTO_GPIO1);
// // 	mcpwm_gpio_init(MOTOR_MCPWM_UNIT, MCPWM0B, MOTO_GPIO2);  
// // // 步骤三：用mcpwm_init()这个函数为这个mcpwm_unit选定一个定时器，

// // 	mcpwm_config_t pwm_config;
// // // 步骤四：通过 mcpwm_config_t 结构体为定时器设置频率和初始值
// // 	pwm_config.frequency = 1000;    // frequency = 500Hz,
// // 	pwm_config.cmpr_a = 0;    // duty cycle of PWMxA = 0
// // 	pwm_config.cmpr_b = 0;    // duty cycle of PWMxb = 0
// // 	pwm_config.counter_mode = MCPWM_UP_COUNTER;
// // 	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
	
// // // 步骤五：调用mcpwm_init()函数使得配置生效
// // 	mcpwm_init(MOTOR_MCPWM_UNIT, MOTO_TIMER, &pwm_config);    //Configure PWM0A & PWM0B with above settings
// // }

// // void loop(){
// // 	mcpwm_set_signal_low(MOTOR_MCPWM_UNIT, MOTO_TIMER, MCPWM_GEN_A);
// // 	mcpwm_set_duty(MOTOR_MCPWM_UNIT, MOTO_TIMER, MCPWM_GEN_B, 30);
// // 	mcpwm_set_duty_type(MOTOR_MCPWM_UNIT, MOTO_TIMER, MCPWM_GEN_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state 
// // }






// #include <Arduino.h>
// #include "driver/mcpwm_prelude.h"
// // #include "soc/mcpwm_reg.h"
// // #include "soc/mcpwm_struct.h"

// #define NUM_SERVOS 4

// // 1. 指定 4 个舵机的引脚
// const int servo_pins[NUM_SERVOS] = {4, 5, 11, 10};

// // 舵机脉宽参数
// #define SERVO_MIN_PULSEUS 500
// #define SERVO_MAX_PULSEUS 2500

// // 2. 定义结构体保存每路 PWM 的句柄
// typedef struct {
//     mcpwm_oper_handle_t oper;
//     mcpwm_cmpr_handle_t cmp;
//     mcpwm_gen_handle_t gen;
// } servo_channel_t;

// servo_channel_t channels[NUM_SERVOS];

// // ESP32 有2个组，每组最多3个运算器。我们用2个定时器（每组1个）来驱动4个运算器
// mcpwm_timer_handle_t timers[2];

// void setup_mcpwm() {
//     // 3. 配置并创建 2 个定时器 (Group 0 和 Group 1 各一个)
//     mcpwm_timer_config_t timer_config = {
//         .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
//         .resolution_hz = 1000000, // 1 MHz (1 tick = 1 us)
//         .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
//         .period_ticks = 20000,    // 20 ms (50 Hz)
//     };

//     // Group 0 的定时器
//     timer_config.group_id = 0;
//     ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timers[0]));

//     // Group 1 的定时器
//     timer_config.group_id = 1;
//     ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timers[1]));

//     // 4. 循环创建 4 个运算器、比较器和生成器
//     for (int i = 0; i < NUM_SERVOS; i++) {
//         // 前3个通道用 Group 0，第4个通道用 Group 1
//         int group_id = (i < 3) ? 0 : 1;

//         // 创建运算器
//         mcpwm_operator_config_t operator_config = {
//             .group_id = group_id,
//         };
//         ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &channels[i].oper));

//         // 将运算器连接到对应组的定时器
//         ESP_ERROR_CHECK(mcpwm_operator_connect_timer(channels[i].oper, timers[group_id]));

//         // 创建比较器
//         mcpwm_comparator_config_t comparator_config = {
//             .flags = {.update_cmp_on_tep = true}, // 防毛刺
//         };
//         ESP_ERROR_CHECK(mcpwm_new_comparator(channels[i].oper, &comparator_config, &channels[i].cmp));

//         // 创建生成器并绑定指定引脚
//         mcpwm_generator_config_t generator_config = {
//             .gen_gpio_num = (gpio_num_t)servo_pins[i], // 指定引脚
//         };
//         ESP_ERROR_CHECK(mcpwm_new_generator(channels[i].oper, &generator_config, &channels[i].gen));

//         // 设置生成器动作：归零拉高，达到比较值拉低
//         ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(channels[i].gen,
//             MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
//         ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(channels[i].gen,
//             MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, channels[i].cmp, MCPWM_GEN_ACTION_LOW)));
//     }

//     // 5. 使能并启动定时器
//     ESP_ERROR_CHECK(mcpwm_timer_enable(timers[0]));
//     ESP_ERROR_CHECK(mcpwm_timer_start_stop(timers[0], MCPWM_TIMER_START_NO_STOP));
//     ESP_ERROR_CHECK(mcpwm_timer_enable(timers[1]));
//     ESP_ERROR_CHECK(mcpwm_timer_start_stop(timers[1], MCPWM_TIMER_START_NO_STOP));
// }

// // 角度控制函数 (传入舵机编号 0-3，和角度 0-180)
// void set_servo_angle(int servo_idx, float angle) {
//     if (servo_idx < 0 || servo_idx >= NUM_SERVOS) return;
    
//     angle = constrain(angle, 0, 180);
//     uint32_t pulse_us = map(angle, 0, 180, SERVO_MIN_PULSEUS, SERVO_MAX_PULSEUS);
    
//     // 更新对应通道的比较值
//     ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(channels[servo_idx].cmp, pulse_us));
// }

// void setup() {
//     Serial.begin(115200);
    
//     // 初始化 MCPWM
//     setup_mcpwm();
//     Serial.println("4-Channel MCPWM Servo Initialized!");

//     // 初始化角度：全部归中
//     for (int i = 0; i < NUM_SERVOS; i++) {
//         set_servo_angle(i, 90);
//     }
// }

// void loop() {
//     // // === 演示：4个舵机同步运动 ===
//     // Serial.println("All servos to 0 degree");
//     // for (int i = 0; i < NUM_SERVOS; i++) set_servo_angle(i, 0);
//     delay(1000);

//     // Serial.println("All servos to 90 degree");
//     // for (int i = 0; i < NUM_SERVOS; i++) set_servo_angle(i, 90);
//     // delay(1000);

//     // Serial.println("All servos to 180 degree");
//     // for (int i = 0; i < NUM_SERVOS; i++) set_servo_angle(i, 180);
//     // delay(1000);

//     // // === 演示：4个舵机独立运动 ===
//     // Serial.println("Independent movement");
//     // set_servo_angle(0, 30);
//     // delay(300);
//     // set_servo_angle(1, 60);
//     // delay(300);
//     // set_servo_angle(2, 120);
//     // delay(300);
//     // set_servo_angle(3, 150);
//     // delay(1000);

//     // // 回到中点
//     // for (int i = 0; i < NUM_SERVOS; i++) set_servo_angle(i, 90);
//     // delay(1000);
// }

