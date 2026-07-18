/* =============================================================================
 *  trot_gait_esp32.h
 *  -----------------------------------------------------------------------------
 *  从 py-apple-dynamics (PA-Dynamics V7.3) 的 Trot 步态移植到 ESP32 (Arduino)。
 *  原作者: 灯哥 (ream_d@yeah.net)  Py-apple dog project
 *  开源协议: Apache License 2.0
 *  GitHub:  https://github.com/ToanTech/py-apple-dynamics
 *
 *  本文件依赖用户提供的舵机驱动库 mixed_servo.hpp (namespace m_servo):
 *      void setup_servos(const uint8_t* pins, const int16_t* corrections);
 *      void set_angle_90_multi(std::array<int16_t, 8> offs);   // 8 个相对 90 度的偏移
 *      constexpr int TOTAL_SERVO_NUM;                            // = 8
 *
 *  IMU 由用户实现 readIMU(IMUData&) 函数 (见文件末尾), 本文件提供 weak 默认实现
 *  (返回 false, 表示无 IMU, 自稳不生效)。
 *
 *  顶层 API (与原 padog.py 一一对应):
 *      trotMainloop()                 5ms 主循环 (核心入口, 由 task 或手动调用)
 *      trotMove(spd, L, R)            设置运动目标 (速度/左转/右转)
 *      trotHeight(goal)               设置站立高度 (mm)
 *      trotGesture(PIT, ROL, X)       设置姿态目标 (俯仰/滚转/X平移)
 *      trotStable(bool)               开关自稳 (需 IMU)
 *      trotGait(mode)                 切换步态 (仅支持 mode=0 Trot)
 *      trotServoInit(key)             舵机回中 (1) 或正常运行 (0)
 *      trotSetStopRunNode(b)          锁定/解锁 (做动作时屏蔽步态)
 *      trotRelease()                  释放舵机 (调用用户提供的 release callback)
 *      trotStartTask()                启动 FreeRTOS 任务自动调用 mainloop (5ms 周期)
 * =============================================================================
 */
#pragma once

#include <cstdint>
#include <cmath>

/* 用户舵机库 */
#include "mixed_servo.hpp"

namespace padynamics {

/* ============================================================================
 * 1. 配置参数 (对应 config_s.py) — 默认值与原工程一致, 用户可修改 trot.cfg
 *
 *    舵机角度约定 (用户硬件):
 *      - 范围 -90~90, 0 为中位
 *      - 度数越大越向前 (对大腿: 髋前摆; 对小腿: 膝伸直方向)
 *      - 用户舵机库已处理左右镜像, 4 条腿用统一公式
 *      - servos_pin[0..3] = 4 个大腿, servos_pin[4..7] = 4 个小腿
 * ========================================================================== */
struct Config {
    /* —— 舵机中位偏置 (用户 -90~90 约定, 0=中位) ——
     *    用于每条腿单独微调零位 (用户库的 servo_correction[] 已做每舵机修正,
     *    通常这里保持 0 即可; 仅在腿间机械不对称时使用)
     */
    int init_1h_offset = 0;   int init_1s_offset = 0;   // 腿1 大腿/小腿
    int init_2h_offset = 0;   int init_2s_offset = 0;   // 腿2
    int init_3h_offset = 0;   int init_3s_offset = 0;   // 腿3
    int init_4h_offset = 0;   int init_4s_offset = 0;   // 腿4

    /* —— 机构尺寸 (mm) —— */
    double l1 = 80;     // 大腿长
    double l2 = 69;     // 小腿长
    double l  = 142;    // 机身长
    double b  = 92.8;   // 机身宽偏置
    double w  = 108;    // 机身宽

    /* —— 步态参数 —— */
    double speed = 0.12;   // 步态相位推进速度 (每次 mainloop 步进值)
    double h     = 45;     // Trot 抬腿高度 (z_target)

    /* —— 调节器增益 —— */
    double Kp_H        = 0.06;   // 高度 P
    double pit_Kp_G    = 0.04;   // 俯仰姿态 P
    double pit_Kd_G    = 0.6;    // 俯仰姿态 D (自稳时用)
    double rol_Kp_G    = 0.04;   // 滚转姿态 P
    double rol_Kd_G    = 0.35;   // 滚转姿态 D (自稳时用)
    double tran_mov_kp = 0.1;    // X 平移 P

    /* —— 机构类型: 0=串联腿, 1=并联腿 (本文件仅实现 0) —— */
    int ma_case = 0;

    /* —— Trot 重心补偿系数 —— */
    double trot_cg_f = 0.5;   // 前进时
    double trot_cg_b = 0.5;   // 后退时
    double trot_cg_t = 0.3;   // 原地转时

    int in_y = 17;          // 自稳 X 偏置基准

    /* —— 限位 —— */
    double pit_max_ang = 25;
    double rol_max_ang = 20;
    double Kp_V        = 1;     // 速度变化 P

    /* —— 小腿机械偏置标定曲线 (原工程 mechan_offset_corr) ——
     *    原工程针对其硬件的非线性标定: mech(x) = 0.006649*x² + 0.4414*x + 5.53
     *    用户硬件若无需此标定 (舵机已线性), 设为 false 即直接用 IK 的 shank 输出
     */
    bool apply_mechan_offset = false;
};

/* ============================================================================
 * 2. IMU 数据结构 (对应 PA_STABLIZE.py 中的相关字段)
 *    用户在自己的 readIMU() 实现里填充此结构。
 * ========================================================================== */
struct IMUData {
    bool   calibrated      = false;   // gyro_cal_sta == 1 (陀螺仪校准完成)
    float  pitch_deg       = 0;       // filter_data_p  俯仰角 (度)
    float  roll_deg        = 0;       // filter_data_r  滚转角 (度)
    float  pitch_origin    = 0;       // p_origin       俯仰零点
    float  roll_origin     = 0;       // r_origin       滚转零点
    float  gyro_x_radps    = 0;       // gyro_x_fitted  俯仰角速度 (rad/s)
    float  gyro_y_radps    = 0;       // gyro_y_fitted  滚转角速度 (rad/s)
    float  acc_z_g         = 0;       // acc_z_fitted   Z 加速度 (g)
};

/* ============================================================================
 * 3. 舵机通道顺序约定 (用户硬件)
 *    用户 servos_pin[8] 顺序:
 *        [0] 腿1 大腿   [1] 腿2 大腿   [2] 腿3 大腿   [3] 腿4 大腿
 *        [4] 腿1 小腿   [5] 腿2 小腿   [6] 腿3 小腿   [7] 腿4 小腿
 *    set_angle_90_multi({a,b,c,d, e,f,g,h}) 中:
 *        a,b,c,d = 4 个大腿角度 (-90~90, 0=中位, 越大越向前)
 *        e,f,g,h = 4 个小腿角度 (-90~90, 0=中位, 越大越向前)
 *    腿编号与原 Python 工程一致 (Trot 对角配对: 1+4 同步, 2+3 同步)。
 *    若你的硬件腿编号顺序不同, 直接在 servo_output 中调换 ham1/2/3/4 即可。
 * ========================================================================== */

/* ============================================================================
 * 4. 状态结构体 (对应 padog.py 中的全局变量, 这里封装成类成员)
 * ========================================================================== */
class TrotController {
public:
    Config cfg;

    /* ---------- 顶层 API (与 padog.py 函数一一对应) ---------- */
    void mainloop();                       // 5ms 主循环 (核心入口)
    void move(double spd, double L, double R);   // 对应 padog.move
    void height(double goal);                    // 对应 padog.height
    void gesture(double PIT, double ROL, double X);   // 对应 padog.gesture
    void stable(bool key);                       // 对应 padog.stable
    void gait(int mode);                         // 对应 padog.gait
    void servoInit(int key);                     // 对应 padog.servo_init
    void setStopRunNode(bool b);                 // 对应 padog.stop_run_node
    void release();                              // 对应 PA_SERVO.release (走回调)

    /* ---------- 调试访问器 ---------- */
    double getT()        const { return state_.t; }
    double getSpd()      const { return state_.spd; }
    double getSpdGoal()  const { return state_.spd_goal; }
    double getRH()       const { return state_.R_H; }
    int    getIKError()  const { return state_.IK_ERROR; }
    int    getGaitMode() const { return state_.gait_mode; }

    /* ---------- 用户可注入: 舵机电源使能回调 (对应 pin_servo_vol.value) ---------- */
    typedef void (*PowerCallback)(bool on);
    void setPowerCallback(PowerCallback cb) { powerCb_ = cb; }

    /* ---------- 用户可注入: 舵机释放回调 (对应 PA_SERVO.release) ---------- */
    /*    原工程通过 PCA9685 把所有通道置 0 实现释放, 用户的 mixed_servo 库
     *    若无等价接口, 可在此回调里自行实现 (例如把所有 PWM 停掉)。
     *    若未设置, release() 仅跳过 servo_output。
     */
    typedef void (*ReleaseCallback)();
    void setReleaseCallback(ReleaseCallback cb) { releaseCb_ = cb; }

private:
    struct State {
        double t = 0;                          // 步态相位 (0..1)
        double init_x = 0, init_y = -110;

        double ges_x_1=0, ges_x_2=0, ges_x_3=0, ges_x_4=0;
        double ges_y_1=0, ges_y_2=0, ges_y_3=0, ges_y_4=0;

        double PIT_S=0, ROL_S=0, X_S=0;
        double PIT_goal=0, ROL_goal=0, X_goal=0;

        double spd=0, spd_goal=0;
        double L=0, R=0;

        double R_H = 110;                      // 当前站立高度
        double H_goal = 110;                   // 目标站立高度

        int  init_case = 0;                    // 0=正常运行, 1=回到中位
        int  gait_mode = 0;                    // 0=Trot (本文件唯一支持)
        bool key_stab  = false;                // 自稳开关
        bool stop_run_node = false;            // 锁定 (做动作时屏蔽步态)

        double speed_init = 0;                 // speed 初始值, 自稳速度补偿
        double acc_z = 0;
        int    IK_ERROR = 0;
        int    normal_node = 0;
        int    error_node = 0;
        double act_tran_mov_kp = 0;

        // PA_STABLIZE 自稳状态
        double Sta_Pitch = 0;
        double Sta_Roll  = 0;
    };

    State state_;
    PowerCallback   powerCb_   = nullptr;
    ReleaseCallback releaseCb_ = nullptr;

    void foward_cg_stab(double r1, double r4, double r2, double r3,
                        double gait_need, bool enable);
    void stab();
    int  alarm_and_servo_control();
    void servo_output(double ham1, double ham2, double ham3, double ham4,
                      double shank1, double shank2, double shank3, double shank4);
};

/* 全局单例 (Arduino 风格, 简化使用) */
extern TrotController trot;

/* ============================================================================
 * 5. C 风格顶层 API (Arduino 友好, 直接调用无需加 trot. 前缀)
 * ========================================================================== */
inline void trotMainloop()                 { trot.mainloop(); }
inline void trotMove(double s, double L, double R) { trot.move(s, L, R); }
inline void trotHeight(double goal)        { trot.height(goal); }
inline void trotGesture(double PIT, double ROL, double X) { trot.gesture(PIT, ROL, X); }
inline void trotStable(bool on)            { trot.stable(on); }
inline void trotGait(int mode)             { trot.gait(mode); }
inline void trotServoInit(int key)         { trot.servoInit(key); }
inline void trotSetStopRunNode(bool b)     { trot.setStopRunNode(b); }
inline void trotRelease()                  { trot.release(); }
inline void trotSetPowerCallback(TrotController::PowerCallback cb)   { trot.setPowerCallback(cb); }
inline void trotSetReleaseCallback(TrotController::ReleaseCallback cb) { trot.setReleaseCallback(cb); }

/* 启动 FreeRTOS 任务自动调用 mainloop (5ms 周期, 默认 core=0, 优先级=4) */
void trotStartTask(uint8_t priority, uint32_t stackSize);

/* 停止 mainloop 任务 */
void trotStopTask();

} // namespace padynamics

/* ============================================================================
 * 6. IMU 读取函数 (用户必须实现, 本文件提供 weak 默认实现返回 false)
 *    用户在自己的 .ino 或 .cpp 中定义同名函数即可覆盖:
 *
 *        bool readIMU(padynamics::IMUData& out) {
 *            // 读你的 MPU6050 / ICM20602 / ...
 *            out.calibrated = true;
 *            out.pitch_deg  = ...;
 *            out.roll_deg   = ...;
 *            ...
 *            return true;   // false 表示 IMU 未就绪
 *        }
 *
 *    参考 PA_STABLIZE.py 中字段含义:
 *        filter_data_p   -> out.pitch_deg       (互补滤波后的俯仰角, 单位度)
 *        filter_data_r   -> out.roll_deg        (互补滤波后的滚转角, 单位度)
 *        p_origin        -> out.pitch_origin    (开机校准的俯仰零点)
 *        r_origin        -> out.roll_origin     (开机校准的滚转零点)
 *        gyro_x_fitted   -> out.gyro_x_radps    (俯仰角速度, 单位 rad/s)
 *        gyro_y_fitted   -> out.gyro_y_radps    (滚转角速度, 单位 rad/s)
 *        acc_z_fitted    -> out.acc_z_g         (Z 轴加速度, 单位 g)
 *        gyro_cal_sta==1 -> out.calibrated      (校准完成标志)
 * ========================================================================== */
bool readIMU(padynamics::IMUData& out);


