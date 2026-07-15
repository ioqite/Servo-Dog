// #include <Arduino.h>
// #include <array>
// #include <ESP32Servo.h>
// #include <string.h>
// #include <WiFi.h>
// #include <esp_wifi.h>
// #include <esp_now.h>

// #define BUTTON_PIN   9

// #define SERVO_NUM    8
// #define SERVO_DELAY  3
// #define SERVO_RETURN_DELAY 50
// //                           大腿:左前 右前 左后 右后 | 膝盖:左前 右前 左后 右后
// uint8_t servos_pin[SERVO_NUM]       = { 4,   5,   7,   6,   10,  8,   3,   2 };
// //                           大腿:左前 右前 左后 右后 | 膝盖:左前 右前 左后 右后
// int servo_correction[SERVO_NUM]     = { 2,  -1,   2,   9,  -1,   0,   0,   0 };
// //                           true=同向  false=反向
// bool servo_local[SERVO_NUM]         = { 0,   1,   0,   1,   0,   1,   0,   1 };
// Servo servos[SERVO_NUM];
// // bool running_demo = false;

// void servo4_demo1();
// void servo4_demo2();
// void trot_forward(int n);
// void walk_forward(int n);
// void stand();
// void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

// // 接收方mac
// // esp32s3-lcd-2
// // const uint8_t mac_target[6] = {0x3c, 0xdc, 0x75, 0x70, 0x0e, 0x94};
// // esp32s3-cam-h
// // const uint8_t mac_target[6] = {0x80, 0xb5, 0x4e, 0xc5, 0xcd, 0x94};
// // esp32-h
// // const uint8_t mac_target[6] = {0x8c, 0x4f, 0x00, 0xb0, 0xfb, 0x78};
// const uint8_t mac_target_ak2[6] = {0xb8, 0xf8, 0x62, 0xed, 0x5a, 0x98};

// // #define check_key 0b10100111

// // 消息传递载体
// struct send_key_t {
// 	char key[40];
// 	// size_t size = sizeof(key);
// 	uint8_t check;
// };
// send_key_t send_key;

// std::array<int8_t, SERVO_NUM> nums;
// std::array<int, SERVO_NUM> angles;
// bool is_trot_forward = false;
// bool is_walk_forward = false;
// uint8_t current_posture = 0;
// enum POSTURE {
// 	POSTURE_NONE,
// 	POSTURE_TROT,
// 	POSTURE_WALK,
// 	POSTURE_SIT,
// 	POSTURE_STRETCH,
// 	POSTURE_LIE_DOWN,
// 	POSTURE_WAVE,
// 	POSTURE_PLAY,

// 	POSTURE_STAND = 12,
// };

// void setup_espnow() {
// 	// 初始化 ESP-NOW
// 	WiFi.mode(WIFI_STA);
// 	// if (esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE) != ESP_OK) Serial.println("Failed to set Wi-Fi channel");

// 	if (esp_now_init() != ESP_OK) {
// 		Serial.println("Error init ESP-NOW");
// 		while (1) delay(1000000);
// 	}
// 	esp_now_register_recv_cb(OnDataRecv);

// 	// esp_now_peer_info_t peerInfo;
// 	// memcpy(peerInfo.peer_addr, mac_target, ESP_NOW_ETH_ALEN);
// 	// // peerInfo.channel = 11; // 设置为固定信道
// 	// peerInfo.ifidx = WIFI_IF_STA;
// 	// peerInfo.encrypt = false;
// 	// if (esp_now_add_peer(&peerInfo) != ESP_OK) Serial.println("add peer Fail");
// 	Serial.println("Init ESP-NOW OK");
// }

// // 数据接收时的回调函数
// void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
// 	// check
// 	memcpy(&send_key, incomingData, sizeof(send_key));
// 	if (memcmp(mac, mac_target_ak2, 6) == 0) {
// 	// if (send_key.check == check_key) {
// 		// while(send_key_lock) vTaskDelay(1 / portTICK_PERIOD_MS);
// 		// send_key_lock = 1;
		
// 		// send_key_lock = 0;
		
// 		String proc_key = send_key.key;
// 		Serial.printf("[incoming key: %s]\r\n", send_key.key);

// 		if (send_key.key[0] == '$' || send_key.key[1] > '0' || send_key.key[1] <= '9') {
// 			current_posture = proc_key.substring(1).toInt();
// 			Serial.printf("current_posture: %d\r\n", current_posture);
// 		} else {
// 			Serial.printf("Unknown command: %s\r\n", send_key.key);
// 			return;
// 		}
// 	}
// 	else {
// 		// Serial.println(String("[check fail], key: ") + String(send_key.key) + String(", check: ") + String(send_key.check));
// 		char macStr[32];
// 		//把接收方mac地址复制为字符串
// 		snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
// 				mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
// 		Serial.print("[check fail]: ");
// 		Serial.println(macStr);
// 	}
// }

// // ============= 基础控制 =============
// void set_angle(int8_t servo_index, int angle) {
// 	if (servo_local[servo_index]) {
// 		servos[servo_index].write(angle + servo_correction[servo_index]);
// 	} else {
// 		servos[servo_index].write(180 - angle + servo_correction[servo_index]);
// 	}
// }

// void set_angle_90(int8_t servo_index, int angle) {
// 	if (servo_local[servo_index]) {
// 		servos[servo_index].write((angle + 90) + servo_correction[servo_index]);
// 	} else {
// 		servos[servo_index].write(180 - (angle + 90) + servo_correction[servo_index]);
// 	}
// }

// void to_angle(int8_t servo_index, int angle) {
//     int tmp = servos[servo_index].read();
//     if (angle > tmp) {
//         for (int agl = tmp; agl <= angle; agl++) {
//             set_angle(servo_index, agl);
//             delay(SERVO_DELAY);
//         }
//     } else {
//         for (int agl = tmp; agl >= angle; agl--) {
//             set_angle(servo_index, agl);
//             delay(SERVO_DELAY);
//         }
//     }
// }

// void to_angle_90(int8_t servo_index, int angle) {
//     int tmp = servos[servo_index].read() - 90;  // 修正：转为偏移量
//     if (angle > tmp) {
//         for (int agl = tmp; agl <= angle; agl++) {
//             set_angle_90(servo_index, agl);
//             delay(SERVO_DELAY);
//         }
//     } else {
//         for (int agl = tmp; agl >= angle; agl--) {
//             set_angle_90(servo_index, agl);
//             delay(SERVO_DELAY);
//         }
//     }
// }

// // 同时将多个舵机 设置到 目标角度(相对90度偏移)
// void set_angle_90_multi(std::array<int8_t, SERVO_NUM> nums, std::array<int, SERVO_NUM> targets, int8_t count) {
//     for (int i = 0; i < count; i++) {
// 		set_angle_90(nums[i], targets[i]);
// 	}
// }

// // 同时将多个舵机平滑地转到目标角度(相对90度偏移)
// void to_angle_90_sync(std::array<int8_t, SERVO_NUM> nums, std::array<int, SERVO_NUM> targets, int8_t count) {
//     int cur[SERVO_NUM];
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

// // ================= 步态代码 (使用同步移动) =================
// void stand() {
//     //              大腿: 0  1  2  3   膝盖: 4  5  6  7
//     nums   = {          0, 1, 2, 3,        4, 5, 6, 7};
//     angles = {          0, 0, 0, 0,        0, 0, 0, 0};
//     // to_angle_90_sync(nums, angles, 8);
//     set_angle_90_multi(nums, angles, 8);
// }

// // ============= Trot 步态 =============
// #define TROT_THIGH   30  // 大腿摆动幅度
// #define TROT_KNEE    30  // 膝盖弯曲幅度(抬脚)
// #define TROT_PAUSE   290 // 两阶段间短暂停顿(ms), 让身体稳定

// void trot_forward(int n) {
// 	for (int i = 0; i < n; i++) {
// 		// 阶段1: 对角组A(左前+右后)前摆, 组B中立支撑
// 		//   大腿:  0   1   2   3    膝盖: 4   5   6   7
// 		nums   = { 0,  1,  2,  3,         4,  5,  6,  7};
// 		angles = {+TROT_THIGH, 0, 0, +TROT_THIGH,  +TROT_KNEE, 0, 0, +TROT_KNEE};
// 		set_angle_90_multi(nums, angles, 8);
// 		delay(TROT_PAUSE);

// 		// 阶段2: 对角组B(右前+左后)前摆, 组A中立支撑
// 		//             大腿: 0   1   2     3  膝盖: 4  5   6   7
// 		nums   = {          0,  1,  2,  3,         4,  5,  6,  7};
// 		angles = {          0,+TROT_THIGH,+TROT_THIGH, 0, 0,+TROT_KNEE,+TROT_KNEE, 0};
// 		set_angle_90_multi(nums, angles, 8);
// 		delay(TROT_PAUSE);
// 	}
// }

// // ============= Walk 步态 =============
// #define WALK_THIGH   40   // 大腿前摆幅度
// #define WALK_KNEE    30   // 膝盖弯曲幅度(抬脚)
// #define WALK_LEG_DT  250  // 每条腿动作后的停顿(ms)

// void walk_forward(int n) {
// 	for (int i = 0; i < n; i++) {
// 		// 1. 左前(0)前摆，其余3腿中立支撑
// 		//             大腿: 0   1  2  3 膝盖: 4  5  6  7
// 		nums   = {          0,  1, 2, 3,     4, 5, 6, 7};
// 		angles = {+WALK_THIGH,          0, 0, 0, +WALK_KNEE,          0, 0, 0};
// 		set_angle_90_multi(nums, angles, 8);
// 		delay(WALK_LEG_DT);

// 		// 2. 右后(3)前摆，其余3腿中立支撑
// 		//              大腿: 0  1  2  3    膝盖: 4  5  6  7
// 		nums   = {          0, 1, 2, 3,         4, 5, 6, 7};
// 		angles = {          0, 0, 0,+WALK_THIGH,           0, 0, 0,+WALK_KNEE};
// 		set_angle_90_multi(nums, angles, 8);
// 		delay(WALK_LEG_DT);

// 		// 3. 右前(1)前摆，其余3腿中立支撑
// 		//              大腿: 0  1           2  3    膝盖: 4  5           6  7
// 		nums   = {          0, 1,           2, 3,         4, 5,           6, 7};
// 		angles = {          0,+WALK_THIGH,  0, 0,         0,+WALK_KNEE,   0, 0};
// 		set_angle_90_multi(nums, angles, 8);
// 		delay(WALK_LEG_DT);

// 		// 4. 左后(2)前摆，其余3腿中立支撑
// 		//              大腿: 0  1  2           3    膝盖: 4  5  6           7
// 		nums   = {          0, 1, 2,           3,         4, 5, 6,           7};
// 		angles = {          0, 0,+WALK_THIGH,  0,         0, 0,+WALK_KNEE,   0};
// 		set_angle_90_multi(nums, angles, 8);
// 		delay(WALK_LEG_DT);
// 	}
// }

// // ============== 自定义姿态 ==============

// // 坐下
// void sit() {
// 	//              大腿: 0    1    2   3    膝盖: 4    5    6   7
// 	nums   = {          0,   1,   2,  3,         4,   5,   6,  7};
// 	angles = {        -25, -25,  40, 40,       -35, -35, -35,-35};
// 	// to_angle_90_sync(nums, angles, 8);
// 	set_angle_90_multi(nums, angles, 8);
// }

// // 拉伸
// void stretch() {
// 	//              大腿: 0   1    2    3    膝盖: 4  5  6  7
// 	nums   = {          0,  1,   2,   3,         4, 5, 6, 7};
// 	angles = {        +30,+30, -30, -30,         0, 0, 0, 0};
// 	set_angle_90_multi(nums, angles, 8);
	
// 	delay(500);

// 	nums   = {          0,  1,   2,   3,         4,  5,  6,  7};
// 	angles = {        +45,+45, -60, -60,       -20,-20,-20,-20};
// 	set_angle_90_multi(nums, angles, 8);
// }

// // 躺下
// void lie_down() {
// 	//              大腿: 0   1   2  3    膝盖: 4  5  6  7
// 	nums   = {          0,  1,  2, 3,         4, 5, 6, 7};
// 	angles = {        +60,+60,  0, 0,         0, 0, 0, 0};
// 	set_angle_90_multi(nums, angles, 8);
	
// 	delay(500);

// 	nums   = {          0,  1,  2,  3,         4,  5,  6,  7};
// 	angles = {        +60,+60,+45,+45,       -30,-30,-30,-30};
// 	set_angle_90_multi(nums, angles, 8);
// }

// // 挥手
// void wave() {
// 	sit();
// 	delay(900);
// 	for (int i = 0; i < 3; i++) {
// 		nums   = {1, 5};
// 		angles = {80, -60};
// 		set_angle_90_multi(nums, angles, 2);
// 		delay(200);
// 		nums   = {1, 5};
// 		angles = {50, -30};
// 		set_angle_90_multi(nums, angles, 2);
// 		delay(200);
// 	}
// 	// 回到坐姿
// 	nums   = {1, 5};
// 	angles = {-25, -35};
// 	set_angle_90_multi(nums, angles, 2);
// }

// // 玩耍
// void play() {
// 	srand(time(NULL));
// 	for (int i = 0; i < 12; i++) {
// 		nums   = {          0,  1,  2,  3,         4,  5,  6,  7};
// 		angles = {
// 			rand() % 80 - 40, rand() % 80 - 40,
// 			rand() % 80 - 40, rand() % 80 - 40,
// 			rand() % 80 - 40, rand() % 80 - 40,
// 			rand() % 80 - 40, rand() % 80 - 40
// 		};
// 		set_angle_90_multi(nums, angles, 8);
// 		delay(200);
// 	}
// }

// // ============= 主循环 =============
// void loop() {
// 	switch (current_posture) {
// 		case POSTURE_STAND:
// 			stand();
// 			break;
// 		case POSTURE_TROT:
// 			trot_forward(1);
// 			current_posture = POSTURE_STAND;
// 			break;
// 		case POSTURE_WALK:
// 			walk_forward(1);
// 			current_posture = POSTURE_STAND;
// 			break;
// 		case POSTURE_SIT:
// 			sit();
// 			// current_posture = POSTURE_STAND;
// 			break;
// 		case POSTURE_STRETCH:
// 			stretch();
// 			// current_posture = POSTURE_STAND;
// 			break;
// 		case POSTURE_LIE_DOWN:
// 			lie_down();
// 			// current_posture = POSTURE_STAND;
// 			break;
// 		case POSTURE_WAVE:
// 			wave();
// 			current_posture = POSTURE_SIT;
// 			break;
// 		case POSTURE_PLAY:
// 			play();
// 			current_posture = POSTURE_STAND;
// 			break;
// 		default:
// 			current_posture = POSTURE_STAND;
// 			break;
// 	}

// 	// if (is_trot_forward) {
// 	// 	is_trot_forward = false;
// 	// 	trot_forward(1);
// 	// } else if (is_walk_forward) {
// 	// 	is_walk_forward = false;
// 	// 	walk_forward(1);
// 	// } else {
// 	// 	stand();
// 	// }

// 	// if (running_demo) {
// 	// 	servo4_demo1();
// 	// 	servo4_demo2();
// 		// stand();
// 		// delay(500);
// 		// trot_forward(100);
// 		// walk_forward(10);
// 		// delay(500);
// 		// stand();

// 	// } else {
// 	// 	stand();
// 	// 	while (!running_demo) {}
// 	// }
// }

// // void handle_button_press() {
// // 	delay(25);
// // 	if (digitalRead(BUTTON_PIN) == LOW) running_demo = !running_demo;
// // }

// void setup() {
// 	for (int8_t i = 0; i < SERVO_NUM; i++) {
// 		servos[i].attach(servos_pin[i]);
// 		set_angle(i, 90);
// 	}
// 	// pinMode(BUTTON_PIN, INPUT_PULLUP);
// 	setup_espnow();
// 	// attachInterrupt(BUTTON_PIN, handle_button_press, FALLING);
// }

// // void servo4_demo1() {
// // 	for(int angle = 0; angle < 180; angle++) {
// // 		for (int8_t i = 0; i < SERVO_NUM; i++) {
// // 			set_angle(i, angle);
// // 		}
// // 		delay(SERVO_DELAY);
// // 	}
// // 	delay(SERVO_RETURN_DELAY);
// // 	for(int angle = 180; angle > 0; angle--) {
// // 		for (int8_t i = 0; i < SERVO_NUM; i++) {
// // 			set_angle(i, angle);
// // 		}
// // 		delay(SERVO_DELAY);
// // 	}
// // 	delay(SERVO_RETURN_DELAY);
// // }

// // void servo4_demo2() {
// // 	while (running_demo) {
// // 		set_angle_90(1, 0);
// // 		set_angle_90(2, 0);
// // 		to_angle_90 (0, 60);
// // 		delay(90);
// // 		to_angle_90 (3, 60);
// // 		delay(SERVO_DELAY);
// // 		
// // 		for(int agl = 0; agl > -60; agl--) {
// // 			set_angle_90(1, agl);
// // 			set_angle_90(2, agl);
// // 			set_angle_90(0, agl + 60);
// // 			set_angle_90(3, agl + 60);
// // 			delay(SERVO_DELAY);
// // 		}
// // 
// // 		delay(200);
// // 	}
// // }