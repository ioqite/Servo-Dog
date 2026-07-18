#include <Arduino.h>
#include <esp_sleep.h>
#include "mixed_servo.hpp"

using namespace m_servo;

#define SLEEP_BTN                   9   // 睡眠按键 引脚
#define SERVO_DELAY                 7     // 舵机延迟，毫秒
#define SERVO_RETURN_DELAY          90    // 舵机返回延迟，毫秒
// 舵机引脚
uint8_t servos_pin[TOTAL_SERVO_NUM]       = {4,  5, 11, 10,      14, 12,  8,  3};
int16_t servo_correction[TOTAL_SERVO_NUM] = {3, -3,  2, 10,      -1,  0,  0,  0};

void setup() {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    // 按下按键 -> 进入深度睡眠
    pinMode(SLEEP_BTN, INPUT_PULLUP);
	attachInterrupt(SLEEP_BTN, [] { esp_deep_sleep_start(); }, FALLING);

    Serial.begin(115200);
    Serial.println("Setup");
    // 初始化舵机
    setup_servos(servos_pin, servo_correction);
}

void loop() {
    for (int16_t i = -30; i < 30; i++) {
        set_angle_90_multi({  i, i, i, i,   i, i, i, i });
        vTaskDelay(SERVO_DELAY / portTICK_PERIOD_MS);
    }
    vTaskDelay(SERVO_RETURN_DELAY / portTICK_PERIOD_MS);
    for (int16_t i = 30; i > -30; i--) {
        set_angle_90_multi({  i, i, i, i,   i, i, i, i });
        vTaskDelay(SERVO_DELAY / portTICK_PERIOD_MS);
    }
    vTaskDelay(SERVO_RETURN_DELAY / portTICK_PERIOD_MS);
}

