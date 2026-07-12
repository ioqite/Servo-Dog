// #include <Arduino.h>
// #include <ESP32Servo.h>

// #define BUTTON_PIN   9

// #define SERVO_NUM    4
// #define SERVO_DELAY  5
// #define SERVO_RETURN_DELAY 50
// uint8_t servos_pin[SERVO_NUM]       = {4,   5,  7,  6};
// //                                    左前 右前 左后 右后
// int16_t servo_correction[SERVO_NUM] = {2, -1,  2,  9};
// bool servo_local[SERVO_NUM]         = {0,   1,   0,   1};
// Servo servos[SERVO_NUM];
// bool running_demo = false;

// void set_angle(int8_t servo_index, int16_t angle) {
// 	if (servo_local[servo_index]) {
// 		servos[servo_index].write(angle + servo_correction[servo_index]);
// 	} else {
// 		servos[servo_index].write(180 - angle + servo_correction[servo_index]);
// 	}
// }

// void set_angle_90(int8_t servo_index, int16_t angle) {
// 	if (servo_local[servo_index]) {
// 		servos[servo_index].write((angle + 90) + servo_correction[servo_index]);
// 	} else {
// 		servos[servo_index].write(180 - (angle + 90) + servo_correction[servo_index]);
// 	}
// }

// // ============ 修复后的原函数 ============

// void to_angle(int8_t servo_index, int16_t angle) {
//     int16_t tmp = servos[servo_index].read();
//     if (angle > tmp) {
//         for (int16_t agl = tmp; agl <= angle; agl++) {
//             set_angle(servo_index, agl);
//             delay(SERVO_DELAY);
//         }
//     } else {
//         for (int16_t agl = tmp; agl >= angle; agl--) {
//             set_angle(servo_index, agl);
//             delay(SERVO_DELAY);
//         }
//     }
// }

// void to_angle_90(int8_t servo_index, int16_t angle) {
//     int16_t tmp = servos[servo_index].read() - 90;  // 修正：转为偏移量
//     if (angle > tmp) {
//         for (int16_t agl = tmp; agl <= angle; agl++) {
//             set_angle_90(servo_index, agl);
//             delay(SERVO_DELAY);
//         }
//     } else {
//         for (int16_t agl = tmp; agl >= angle; agl--) {
//             set_angle_90(servo_index, agl);
//             delay(SERVO_DELAY);
//         }
//     }
// }

// // ============ 新增：多舵机同步移动 ============

// // 同时将多个舵机平滑地转到目标角度(相对90度偏移)
// // 每步所有舵机各走1度, 保证动作同步
// void to_angle_90_sync(int8_t nums[], int16_t targets[], int8_t count) {
//     int16_t cur[4];
//     for (int8_t i = 0; i < count; i++)
//         cur[i] = servos[nums[i]].read() - 90;

//     bool moving = true;
//     while (moving) {
//         moving = false;
//         for (int8_t i = 0; i < count; i++) {
//             if (cur[i] < targets[i]) { cur[i]++; moving = true; }
//             else if (cur[i] > targets[i]) { cur[i]--; moving = true; }
//             set_angle_90(nums[i], cur[i]);
//         }
//         if (moving) delay(SERVO_DELAY);
//     }
// }

// // ============ 步态代码 (使用同步移动) ============

// #define SWING   25
// #define PAUSE   80  // 两阶段间短暂停顿(ms), 让身体稳定

// void stand() {
//     int8_t  all[] = {0, 1, 2, 3};
//     int16_t mid[] = {0, 0, 0, 0};
//     to_angle_90_sync(all, mid, 4);
// }

// void step_forward() {
//     // 阶段1: 组A前摆 + 组B后蹬 (4腿同时动)
//     int8_t  n1[] = {0, 3, 1, 2};
//     int16_t t1[] = {SWING, SWING, -SWING, -SWING};
//     to_angle_90_sync(n1, t1, 4);
//     delay(PAUSE);

//     // 阶段2: 组B前摆 + 组A后蹬 (4腿同时动)
//     int8_t  n2[] = {1, 2, 0, 3};
//     int16_t t2[] = {SWING, SWING, -SWING, -SWING};
//     to_angle_90_sync(n2, t2, 4);
//     delay(PAUSE);
// }

// void walk_forward(int n) {
//     for (int i = 0; i < n; i++)
//         step_forward();
// }

// void loop() {
// 	if (running_demo) {
// 		// servo4_demo1();
// 		// servo4_demo2();
// 		stand();
// 		delay(500);
// 		walk_forward(10);
// 		stand();

// 	} else {
// 		stand();
// 		while (!running_demo) {}
// 	}
// }

// void handle_button_press() {
// 	delay(25);
// 	if (digitalRead(BUTTON_PIN) == LOW) running_demo = !running_demo;
// }

// void setup() {
// 	for (int8_t i = 0; i < SERVO_NUM; i++) {
// 		servos[i].attach(servos_pin[i]);
// 		set_angle(i, 90);
// 	}
// 	pinMode(BUTTON_PIN, INPUT_PULLUP);
// 	attachInterrupt(BUTTON_PIN, handle_button_press, FALLING);
// }

// void servo4_demo1() {
// 	for(int16_t angle = 0; angle < 180; angle++) {
// 		for (int8_t i = 0; i < SERVO_NUM; i++) {
// 			set_angle(i, angle);
// 		}
// 		delay(SERVO_DELAY);
// 	}
// 	delay(SERVO_RETURN_DELAY);
// 	for(int16_t angle = 180; angle > 0; angle--) {
// 		for (int8_t i = 0; i < SERVO_NUM; i++) {
// 			set_angle(i, angle);
// 		}
// 		delay(SERVO_DELAY);
// 	}
// 	delay(SERVO_RETURN_DELAY);
// }

// void servo4_demo2() {
// 	while (running_demo) {
// 		set_angle_90(1, 0);
// 		set_angle_90(2, 0);

// 		to_angle_90 (0, 60);
// 		delay(90);
// 		to_angle_90 (3, 60);
// 		delay(SERVO_DELAY);
		
// 		for(int16_t agl = 0; agl > -60; agl--) {
// 			set_angle_90(1, agl);
// 			set_angle_90(2, agl);
// 			set_angle_90(0, agl + 60);
// 			set_angle_90(3, agl + 60);
// 			delay(SERVO_DELAY);
// 		}

// 		delay(200);
// 	}
// }
