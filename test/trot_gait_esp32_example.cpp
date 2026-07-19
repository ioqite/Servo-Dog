/* =============================================================================
 *  trot_gait_esp32_example.ino
 *  -----------------------------------------------------------------------------
 *  与用户现有 mixed_servo 工程的 setup/loop 风格对齐的示例 sketch。
 *
 *  用户硬件约定:
 *    - 舵机角度 -90~90, 0=中位, 度数越大越向前
 *    - servos_pin[0..3] = 4 个大腿, servos_pin[4..7] = 4 个小腿
 *    - 用户舵机库已处理左右镜像
 *    - 腿编号与原 Python 工程一致 (Trot 对角配对: 1+4 同步, 2+3 同步)
 *
 *  功能:
 *    - 保留用户原有的 SLEEP_BTN / GPIO13 / setup_servos 等初始化
 *    - 启动 Trot mainloop FreeRTOS 任务 (5ms 周期, 自动调用 trotMainloop)
 *    - 通过串口命令控制运动: f/b/l/r/s/q (前进/后退/左转/右转/停止/释放)
 *    - 预留 readIMU() 由用户实现 (默认返回 false, 自稳不生效)
 * =============================================================================
 */
#include <Arduino.h>
#include <esp_sleep.h>
#include "mixed_servo.hpp"
#include "trot_gait_esp32.h"

using namespace m_servo;
using namespace padynamics;

#define SLEEP_BTN                   9     // 睡眠按键 引脚
// #define SERVO_DELAY                 7     // 舵机延迟，毫秒 (保留, demo 中未用)
// #define SERVO_RETURN_DELAY          90    // 舵机返回延迟，毫秒 (保留, demo 中未用)

// 舵机引脚
uint8_t servos_pin[TOTAL_SERVO_NUM]       = { 4,  5, 11, 10,      14, 12,  0,  3};
// 舵机校准
int16_t servo_correction[TOTAL_SERVO_NUM] = { 5, -3,  2,  3,       6, -1,  2, -7};

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
// 对应原工程的 PA_SERVO.release(), 即把所有 PWM 通道关掉。
// 用户的 mixed_servo 库若无等价接口, 可在此调用底层 API 自行实现。
void onReleaseServos() {
    // 例如: 把所有舵机设为中位 (相对安全的姿态)
    m_servo::set_angle_90_multi({ 0, 0, 0, 0, 0, 0, 0, 0 });
    // 或: 调用 mixed_servo 库提供的断电接口 (若有)
}

// ------------------ 舵机电源使能回调 (可选) ------------------
// 对应原工程的 pin_servo_vol.value(1/0), 由用户根据硬件实现。
// 例如: 控制 PMOS / EN 引脚通断舵机电源。
// 这里用 GPIO13 做示意 (用户原 setup 里已置 HIGH, 这里仅演示接口)。
void onServoPower(bool on) {
    // digitalWrite(13, on ? HIGH : LOW);
    // Serial.println("Enable Servo Power");
}

void setup() {
    // ---- 用户原有初始化 ----
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    pinMode(SLEEP_BTN, INPUT_PULLUP);
    attachInterrupt(SLEEP_BTN, [] { esp_deep_sleep_start(); }, FALLING);

    Serial.begin(115200);
    Serial.println("Setup");

    // 初始化舵机
    setup_servos(servos_pin, servo_correction);

    // ---- Trot 步态初始化 ----
    // (可选) 注入电源/释放回调
    trotSetPowerCallback(onServoPower);
    trotSetReleaseCallback(onReleaseServos);

    // 设置初始参数
    trotGait(0);          // Trot 步态
    trotHeight(110);      // 站立高度 110mm (与原工程一致)
    trotServoInit(0);     // 正常运行模式 (1=回中)
    trotStable(false);    // 默认关闭自稳

    // 用户硬件可选配置 (默认值通常无需修改):
    // trot.cfg.apply_mechan_offset = false;  // true=启用原工程小腿非线性标定曲线
    // trot.cfg.init_1h_offset = 0;           // 每条腿单独微调零位 (用户库 servo_correction[] 已修正, 通常保持 0)

    // 启动 5ms 周期 mainloop 任务 (core=0, 优先级=4)
    trotStartTask(4, 4096);

    Serial.println("Trot gait started. Commands: f/b/l/r/s/q");
    Serial.println("  f = forward, b = backward, l = turn left, r = turn right");
    Serial.println("  s = stop,    q = release servos");
}

void loop() {
    // 串口命令控制 (mainloop 在 FreeRTOS 任务里跑, 这里只做控制输入)
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toLowerCase();
        if (cmd == "f") {
            trotServoInit(0);
            trotStartTask(4, 4096);         // 确保任务在运行 (q 后重启)
            trotMove(2.0, 1, 1);          // 前进
            Serial.println("-> forward");
        } else if (cmd == "b") {
            trotServoInit(0);
            trotStartTask(4, 4096);
            trotMove(2.0, -1, -1);        // 后退
            Serial.println("-> backward");
        } else if (cmd == "l") {
            trotServoInit(0);
            trotStartTask(4, 4096);
            trotMove(2.5, -1, 1);         // 原地左转
            Serial.println("-> turn left");
        } else if (cmd == "r") {
            trotServoInit(0);
            trotStartTask(4, 4096);
            trotMove(2.5, 1, -1);         // 原地右转
            Serial.println("-> turn right");
        } else if (cmd == "s") {
            trotMove(0, 0, 0);            // 停止
            Serial.println("-> stop");
        } else if (cmd == "q") {
            trotStopTask();
            trotRelease();
            Serial.println("-> released");
        } else if (cmd == "h") {
            trotHeight(110);
            Serial.println("-> height=110");
        } else if (cmd == "low") {
            trotHeight(90);
            Serial.println("-> height=90");
        } else if (cmd == "hi") {
            trotHeight(130);
            Serial.println("-> height=130");
        } else if (cmd == "stab_on") {
            trotStable(true);
            Serial.println("-> trot Stable ON");
        } else if (cmd == "stab_off") {
            trotStable(false);
            Serial.println("-> trot Stable OFF");
        } else if (cmd == "info") {
            Serial.printf("t=%.3f spd=%.3f spd_goal=%.3f R_H=%.1f IK_ERR=%d\n",
                          trot.getT(), trot.getSpd(), trot.getSpdGoal(),
                          trot.getRH(),  trot.getIKError());
        }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
}
