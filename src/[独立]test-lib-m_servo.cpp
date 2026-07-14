// #include <Arduino.h>
// #include "mixed_servo.hpp"

// using namespace m_servo;

// #define SERVO_DELAY  5
// #define SERVO_RETURN_DELAY 50
// // //                                    左前 右前 左后 右后
// // uint8_t servos_pin[TOTAL_SERVO_NUM]       = {4,  5,  7,  6};
// // // uint8_t servos_pin[TOTAL_SERVO_NUM]       = {10,  8,  3,  2};
// // int16_t servo_correction[TOTAL_SERVO_NUM] = {2, -1,  2,  9};
// // // int16_t servo_correction[TOTAL_SERVO_NUM] = {-1, 0,  0,  0};
// // bool servo_local[TOTAL_SERVO_NUM]         = {0,  1,  0,  1};

// //                                         左前 右前 左后 右后
// uint8_t servos_pin[TOTAL_SERVO_NUM]       = {4,  5, 11, 10,      14, 12,  8,  3};
// int16_t servo_correction[TOTAL_SERVO_NUM] = {2, -1,  2,  9,      -1,  0,  0,  0};


// void setup() {
//     pinMode(13, OUTPUT);
//     digitalWrite(13, HIGH);

//     Serial.begin(115200);
//     Serial.println("Setup");
//     // 初始化舵机
//     setup_servos(servos_pin, servo_correction);
// }

// void loop() {
//     for (int16_t i = -30; i < 30; i++) {
//         set_angle_90_multi({  i, i, i, i,   i, i, i, i });
//         vTaskDelay(SERVO_DELAY / portTICK_PERIOD_MS);
//     }
//     vTaskDelay(SERVO_RETURN_DELAY / portTICK_PERIOD_MS);
//     for (int16_t i = 30; i > -30; i--) {
//         set_angle_90_multi({  i, i, i, i,   i, i, i, i });
//         vTaskDelay(SERVO_DELAY / portTICK_PERIOD_MS);
//     }
//     vTaskDelay(SERVO_RETURN_DELAY / portTICK_PERIOD_MS);
// }

