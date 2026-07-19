#include <Arduino.h>
#include <esp_sleep.h>
#include <array>
#include <string.h>
#include "mixed_servo.hpp"
#include "BLETextLink.hpp"
#include "user_confug.h"
// #include "TinyWS2812.hpp"
#include "Adafruit_NeoPixel.h"
#include "padog_gait.h"

using namespace m_servo;
using namespace padynamics;

#define SLEEP_BTN                   9   // 睡眠按键 引脚
#define SERVO_DELAY                 7     // 舵机延迟，毫秒
#define SERVO_RETURN_DELAY          90    // 舵机返回延迟，毫秒
#define LEG_OFFSET     30 // n: 大腿舵机偏移
#define KNEE_OFFSET   -50 // n: 小腿舵机偏移
// 舵机引脚
uint8_t servos_pin[TOTAL_SERVO_NUM]       = { 4,  5, 11, 10,      14, 12,  0,  3};
// 舵机校准
int16_t servo_correction[TOTAL_SERVO_NUM] = { 5, -3,  2,  3,       6, -1,  2, -7};

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
	POSTURE_N_TROT_FORWARD,
	POSTURE_HALF_STAND,

	POSTURE_STAND = 12,
};

// WS2812 pixels(8, 1);
Adafruit_NeoPixel pixels(1, 8, NEO_GRB + NEO_KHZ800);

// 蓝牙(BLE) 传输器
BLETextLink bleLink;

// bool running_demo = false;
// bool is_trot_forward = false;
// bool is_walk_forward = false;
// void servo4_demo1();
// void servo4_demo2();
void trot_forward(int n);
void walk_forward(int n);
void stand();
bool readIMU(padynamics::IMUData& out);

// BLE 连接 回调
void BLEonConnect();
// BLE 断开连接 回调
void BLEonDisconnect();
// BLE 接收 回调
void BLEonReceive(const String& msg);

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


// ================================ 步态代码 ================================

// 站立
void stand() {
    set_angle_90_multi({0, 0, 0, 0,   0, 0, 0, 0});
}

// ============= Trot 步态（对角步态，前进） =============
#define TROT_SWING   30
// #define TROT_STANCE  -15
// #define TROT_KNEE_UP 25
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
// #define WALK_KNEE_UP 22
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

void half_stand() {
	set_angle_90_multi({ LEG_OFFSET, LEG_OFFSET, LEG_OFFSET, LEG_OFFSET,   KNEE_OFFSET, KNEE_OFFSET, KNEE_OFFSET, KNEE_OFFSET});
}

void n_trot_forward() {
	trotMoveForward(2.0);      // 前进
	delay(260);
	while (1) {
		bleLink.loop();
		if (current_posture != POSTURE_N_TROT_FORWARD) break;
		bleLink.clearBuffer();
		current_posture = POSTURE_NONE;
		delay(260);
	}
	trotRelease();
}

// // ============== 自定义姿态 ==============

// 坐下
void sit() {
    set_angle_90_multi({ -23, -23, 28, 28,   -9, -9, 29, 29 });
	// 旧步态：{ -25, -25, 39, 39,   0, 0, 0, 0 }
}

// 拉伸
void stretch() {
    set_angle_90_multi({ +41, +41, -10, -10,   30, 30, -12, -12 });
}

// 躺下
void lie_down() {
    set_angle_90_multi({ +41, +41, -41, -41,   0, 0, 0, 0 });
}

// 挥手
void wave() {
	sit();
	delay(460);
	for (int i = 0; i < 3; i++) {
		set_angle_90(1, 90);
		set_angle_90(5, 40);
		//  {0, 90, 0, 0, 0, 0, 20, 0};
		delay(200);
		set_angle_90(1, 60);
		set_angle_90(5, 10);
		//  {0, 60, 0, 0, 0, 0, -15, 0};
		delay(200);
	}
	// 回到坐姿
	set_angle_90(1, -25);
	set_angle_90(5, 0);
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
		delay(200);
	}
}

// 处理姿态
void proc_posture(void *arg) {
	if (current_posture == POSTURE_NONE) return;

	uint32_t tmpColor = pixels.getPixelColor(0);

	pixels.setPixelColor(0, 255, 174, 34);// rgb(255, 174, 34)
	pixels.show();
	switch (current_posture) {
		case POSTURE_STAND:
			stand();
			current_posture = POSTURE_NONE;
			break;
		case POSTURE_HALF_STAND:
			half_stand();
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
		case POSTURE_N_TROT_FORWARD:
			n_trot_forward();
			current_posture = POSTURE_STAND;
			break;
		case POSTURE_NONE:
			break;
		default:
			current_posture = POSTURE_NONE;
			break;
	}
	bleLink.clearBuffer();
	delay(20);
	pixels.setPixelColor(0, tmpColor);
	pixels.show();
}

void loop() {
	bleLink.loop();
	proc_posture(nullptr);
	vTaskDelay(20 / portTICK_PERIOD_MS);
	
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


// ####################### BLE 回调 #######################

// BLE 连接 回调
void BLEonConnect() {
    Serial.println("[BLE事件] 已连接 键盘 或 其他设备");
    // 这里可以做"上线后初始化"操作, 如发送握手消息
    // bleLink.send("Connected from " + bleLink.localAddress());
	pixels.setPixelColor(0, 34, 111, 255);// rgb(34, 111, 255)
	pixels.show();
}

// BLE 断开 回调
void BLEonDisconnect() {
    Serial.println("[BLE事件] 连接已断开 对方会自动重连");
	pixels.setPixelColor(0, 89, 255, 34);// rgb(89, 255, 34)
	pixels.show();
}

// BLE 接收 回调
void BLEonReceive(const String& msg) {
    Serial.printf("[BLE事件 | 接收] %u bytes: %s\r\n", msg.length(), msg.c_str());
    // 示例: 收到 "ping" 回 "pong"
    // if (msg == "ping") bleLink.send("pong");

	// 验证消息格式
	if (msg == "") return;
	if (msg[0] != 0x02) return;
	if (msg[msg.length() - 1] != 0x03) return;

	// 解析消息
	String proc_key = msg.substring(1, msg.length() - 1);
	// Serial.printf("[Incoming key: %s]\r\n", proc_key.c_str());

	if (proc_key[0] == '$' && proc_key.substring(1).toInt() > 0) {
		current_posture = proc_key.substring(1).toInt();
		Serial.printf("Posture: %d\r\n", current_posture);
		// proc_posture(nullptr);
	} else {
		Serial.printf("Unknown command: %s\r\n", proc_key.c_str());
		return;
	}
}

void setup() {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    Serial.begin(115200);
    Serial.println("Setup");
	pixels.begin();
	pixels.setBrightness(35);
	pixels.setPixelColor(0, 89, 255, 34);// rgb(89, 255, 34)
	pixels.show();

	// 初始化 BLE 连接
    bleLink.begin(BLE_PEER_MAC, BLE_ROLE, BLE_NAME);

    // 注册回调 (顺序无关, 可在 begin 之后任意时刻注册)
    bleLink.onConnect(BLEonConnect);
    bleLink.onDisconnect(BLEonDisconnect);
    bleLink.onReceive(BLEonReceive);

	Serial.println("======= BLE 信息 =======");
	Serial.printf ("Local BLE MAC: %s\n", bleLink.localAddress().c_str());
    Serial.printf ("Peer  BLE MAC: %s\n", bleLink.peerAddress().c_str());
    Serial.printf ("Role         : %s\n", bleLink.role() == BLETextLink::MASTER ? "MASTER" : "SLAVE");
	Serial.println("=======================");

    // 按下按键 -> 进入深度睡眠
    pinMode(SLEEP_BTN, INPUT_PULLUP);
	attachInterrupt(SLEEP_BTN, [] { pixels.setPixelColor(0, 39, 13, 97); pixels.show(); stand(); esp_deep_sleep_start(); }, FALLING);
    // pinMode(BUTTON_PIN, INPUT_PULLUP);
	// attachInterrupt(BUTTON_PIN, handle_button_press, FALLING);

    // 初始化舵机
    setup_servos(servos_pin, servo_correction);

	// ========= 步态初始化 =========
    // (可选) 注册 电源/释放回调
    // trotSetPowerCallback(onServoPower);
    trotSetReleaseCallback(stand);

    // 设置初始参数
    trotGait(0);          // Trot 步态
    trotHeight(110);      // 站立高度 110mm (与原工程一致)
    trotServoInit(0);     // 正常运行模式 (1=回中)
    trotStable(false);    // 默认关闭自稳
	// 启动 周期 mainloop 任务 (优先级=4)
    trotStartTask(4, 4096);
	trotRelease();
}


// ------------------ IMU 读取函数 (用户实现) ------------------
// 本示例返回 false (无 IMU), 自稳功能将不生效。
// 接入真实 IMU 后, 在此填充 IMUData 各字段并 return true。
bool readIMU(padynamics::IMUData& out) {
    // TODO: 读取你的 IMU (MPU6050 / ICM20602 / ...)
    // 示例 (伪代码):
    //   out.calibrated   = (gyro 校准完成?);
    //   out.pitch_deg    = ...;
    //   out.roll_deg     = ...;
    //   out.pitch_origin = ...;
    //   out.roll_origin  = ...;
    //   out.gyro_x_radps = ...;
    //   out.gyro_y_radps = ...;
    //   out.acc_z_g      = ...;
    //   return true;
    (void)out;
    return false;
}

// ------------------ 舵机释放回调 (可选) ------------------
// void onReleaseServos() {
//     // 例如: 把所有舵机设为中位 (相对安全的姿态)
//     m_servo::set_angle_90_multi({ 0, 0, 0, 0, 0, 0, 0, 0 });
//     // 或: 调用 mixed_servo 库提供的断电接口 (若有)
// }


