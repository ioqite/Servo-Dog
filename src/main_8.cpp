#include <Arduino.h>
#include <esp_sleep.h>
#include <array>
#include <string.h>
#include "mixed_servo.hpp"

using namespace m_servo;

#define SLEEP_BTN                   9   // 睡眠按键 引脚
#define SERVO_DELAY                 7     // 舵机延迟，毫秒
#define SERVO_RETURN_DELAY          90    // 舵机返回延迟，毫秒
// 舵机引脚
uint8_t servos_pin[TOTAL_SERVO_NUM]       = {4,  5, 11, 10,      14, 12,  8,  3};
int16_t servo_correction[TOTAL_SERVO_NUM] = {3, -3,  2, 10,      -1,  0,  0,  0};

uint8_t current_posture = 0;
enum POSTURE {
	POSTURE_NONE,
	POSTURE_TROT,
	POSTURE_WALK,
	POSTURE_SIT,
	POSTURE_LIE_DOWN,
	POSTURE_STRETCH,
	POSTURE_WAVE,
	POSTURE_PLAY,
	POSTURE_JUMP_RUN,

	POSTURE_STAND = 12,
};

// bool running_demo = false;
// bool is_trot_forward = false;
// bool is_walk_forward = false;
// void servo4_demo1();
// void servo4_demo2();
void trot_forward(int n);
void walk_forward(int n);
void stand();

void setup() {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    Serial.begin(115200);
    Serial.println("Setup");

    // 按下按键 -> 进入深度睡眠
    pinMode(SLEEP_BTN, INPUT_PULLUP);
	attachInterrupt(SLEEP_BTN, [] { esp_deep_sleep_start(); }, FALLING);
    // pinMode(BUTTON_PIN, INPUT_PULLUP);
	// attachInterrupt(BUTTON_PIN, handle_button_press, FALLING);

    // 初始化舵机
    setup_servos(servos_pin, servo_correction);
}

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


// ========================== 步态代码 ==========================

void stand() {
    set_angle_90_multi({0, 0, 0, 0,   0, 0, 0, 0});
}

// ============= Trot 步态 =============
#define TROT_SWING   30
#define TROT_PAUSE   270  // 两阶段间短暂停顿(ms), 让身体稳定

void trot_forward(int n) {
	for (int i = 0; i < n; i++) {
		// // 阶段1: 组A前摆 + 组B后蹬 (4腿同时动)
        set_angle_90_multi({ +TROT_SWING, 0, 0, +TROT_SWING,   0, 0, 0, 0});
		//  { +TROT_SWING, 0, 0, +TROT_SWING };
		delay(TROT_PAUSE);

		// // 阶段2: 组B前摆 + 组A后蹬 (4腿同时动)
        set_angle_90_multi({ 0, +TROT_SWING, +TROT_SWING, 0,   0, 0, 0, 0});
		//  { 0, +TROT_SWING, +TROT_SWING, 0 };
		delay(TROT_PAUSE);
	}
}

// ============= Walk 步态 =============
#define WALK_SWING   30
#define WALK_PAUSE   230  // 两阶段间短暂停顿(ms), 让身体稳定

void walk_forward(int n) {
	for (int i = 0; i < n; i++) {
		// 1. 左前(0) 前摆，其余3腿 支撑
        set_angle_90_multi({ +WALK_SWING, 0, 0, 0,   0, 0, 0, 0});
		//  { +WALK_SWING, 0, 0, 0 };
		delay(WALK_PAUSE);

		// 2. 右后(3) 前摆，其余3腿 支撑
        set_angle_90_multi({ 0, 0, 0, +WALK_SWING,   0, 0, 0, 0});
		//  { 0, 0, 0, +WALK_SWING };
		delay(WALK_PAUSE);

		// 3. 右前(1) 前摆，其余3腿 支撑
        set_angle_90_multi({ 0, +WALK_SWING, 0, 0,   0, 0, 0, 0});
		//  { 0, +WALK_SWING, 0, 0 };
		delay(WALK_PAUSE);

		// 4. 左后(2) 前摆，其余3腿 支撑
        set_angle_90_multi({ 0, 0, +WALK_SWING, 0,   0, 0, 0, 0});
		//  { 0, 0, +WALK_SWING, 0 };
		delay(WALK_PAUSE);
	}
}


// // ============== 自定义姿态 ==============

// 坐下
void sit() {
    set_angle_90_multi({ -25, -25, 39, 39,   0, 0, 0, 0 });
	//  { -25, -25, 39, 39 };
}

// 拉伸
void stretch() {
    set_angle_90_multi({ +60, +60, -10, -10,   0, 0, 0, 0 });
	//  { +60, +60, -10, -10 };
}

// 躺下
void lie_down() {
    set_angle_90_multi({ +41, +41, -41, -41,   0, 0, 0, 0 });
	//  { +41,+41, -41, -41 };
}

// 挥手
void wave() {
	sit();
	delay(460);
	for (int i = 0; i < 3; i++) {
		set_angle_90(1, 90);
		//  {0, 90, 0, 0, 0, 0, 20, 0};
		delay(200);
		set_angle_90(1, 60);
		//  {0, 60, 0, 0, 0, 0, -15, 0};
		delay(200);
	}
	// 回到坐姿
	set_angle_90(1, -25);
}

// 玩耍
void play() {
	srand(time(NULL));
	for (int i = 0; i < 8; i++) {
        set_angle_90_multi({
            (int16_t)(rand() % 80 - 40), (int16_t)(rand() % 80 - 40),
            (int16_t)(rand() % 80 - 40), (int16_t)(rand() % 80 - 40),
            (int16_t)(rand() % 80 - 40), (int16_t)(rand() % 80 - 40),
            (int16_t)(rand() % 80 - 40), (int16_t)(rand() % 80 - 40)
        });
		// {
		// 	rand() % 80 - 40,
		// 	rand() % 80 - 40,
		// 	rand() % 80 - 40,
		// 	rand() % 80 - 40
		// };
		delay(200);
	}
}

// 跳跑
void jump_run(int n) {
	for (int i = 0; i < n; i++) {
		sit();
		delay(400);
		stand();
		delay(400);
	}
}

// 处理姿态
void proc_posture(void *arg) {
	switch (current_posture) {
		case POSTURE_STAND:
			stand();
			current_posture = POSTURE_NONE;
			break;
		case POSTURE_TROT:
			trot_forward(1);
			current_posture = POSTURE_STAND;
			break;
		case POSTURE_WALK:
			walk_forward(1);
			current_posture = POSTURE_STAND;
			break;
		case POSTURE_SIT:
			sit();
			current_posture = POSTURE_NONE;
			break;
		case POSTURE_LIE_DOWN:
			lie_down();
			current_posture = POSTURE_NONE;
			break;
		case POSTURE_STRETCH:
			stretch();
			current_posture = POSTURE_NONE;
			break;
		case POSTURE_WAVE:
			wave();
			current_posture = POSTURE_SIT;
			break;
		case POSTURE_PLAY:
			play();
			current_posture = POSTURE_STAND;
			break;
		case POSTURE_JUMP_RUN:
			jump_run(1);
			current_posture = POSTURE_STAND;
			break;
		case POSTURE_NONE:
			break;
		default:
			current_posture = POSTURE_NONE;
			break;
	}
}

void loop() {
    // if (is_trot_forward) {
    // 	is_trot_forward = false;
    // 	trot_forward(1);
    // } else if (is_walk_forward) {
    // 	is_walk_forward = false;
    // 	walk_forward(1);
    // } else {
    // 	stand();
    // }

    // if (running_demo) {
    // 	servo4_demo1();
    // 	servo4_demo2();
    // 		// stand();
    // 		// delay(500);
    // 		// trot_forward(100);
    // 		// walk_forward(10);
    // 		// delay(500);
    // 		// stand();

    // } else {
    // 	stand();
    // 	while (!running_demo) {}
    // }
}


// void handle_button_press() {
	// delay(25);
	// if (digitalRead(BUTTON_PIN) == LOW) running_demo = !running_demo;
// }

// void servo4_demo1() {
	// for(int16_t angle = 0; angle < 180; angle++) {
		// for (int8_t i = 0; i < TOTAL_SERVO_NUM; i++) {
			// set_angle(i, angle);
		// }
		// delay(SERVO_DELAY);
	// }
	// delay(SERVO_RETURN_DELAY);
	// for(int16_t angle = 180; angle > 0; angle--) {
		// for (int8_t i = 0; i < TOTAL_SERVO_NUM; i++) {
			// set_angle(i, angle);
		// }
		// delay(SERVO_DELAY);
	// }
	// delay(SERVO_RETURN_DELAY);
// }

// void servo4_demo2() {
	// while (running_demo) {
		// set_angle_90(1, 0);
		// set_angle_90(2, 0);
        // 
		// to_angle_90 (0, 60);
		// delay(90);
		// to_angle_90 (3, 60);
		// delay(SERVO_DELAY);
		// 
		// for(int16_t agl = 0; agl > -60; agl--) {
			// set_angle_90(1, agl);
			// set_angle_90(2, agl);
			// set_angle_90(0, agl + 60);
			// set_angle_90(3, agl + 60);
			// delay(SERVO_DELAY);
		// }
        //
		// delay(200);
	// }
// }


