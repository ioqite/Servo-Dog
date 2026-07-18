/* =============================================================================
 *  trot_gait_esp32.cpp
 *  -----------------------------------------------------------------------------
 *  Trot 步态 ESP32 (Arduino) 移植实现。数学逻辑与原 Python 工程完全一致,
 *  仅做以下适配:
 *    - 移除 Linux i2c-dev 依赖
 *    - 用 m_servo::set_angle_90_multi 批量下发 8 路舵机
 *    - 用 FreeRTOS task 实现 5ms 周期主循环
 *    - IMU 由用户提供的 readIMU() 注入
 *
 *  编译: 在 Arduino IDE 中选择 ESP32 板卡, 把本文件 + trot_gait_esp32.h
 *        + mixed_servo.hpp 一起放入 sketch 的 src/ 目录即可。
 * =============================================================================
 */

#include "trot_gait_esp32.h"

#include <Arduino.h>
#include <cmath>
#include <algorithm>

namespace padynamics {

/* ============================================================================
 * 数学常量
 * ========================================================================== */
// constexpr double PI = 3.14159265358979323846;
inline double deg2rad(double d) { return d * PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / PI; }

/* ============================================================================
 * 全局单例
 * ========================================================================== */
TrotController trot;

/* ============================================================================
 * 摆动相 / 支撑相曲线生成 (对应 padog.swing_curve_generate /
 *                            padog.support_curve_generate)
 * ========================================================================== */
struct SwingResult   { double xf, zf, x_past, t_past; };
struct SupportResult { double xf, zf; };

static SwingResult swing_curve_generate(double t, double Tf, double xt, double zh,
                                        double x0, double z0, double xv0) {
    double xf = 0.0, zf = 0.0;

    // ---- X Generator ----
    if (t >= 0 && t < Tf / 4) {
        xf = (-4 * xv0 / Tf) * t * t + xv0 * t + x0;
    }
    if (t >= Tf / 4 && t < (3 * Tf) / 4) {
        double Tf2 = Tf * Tf;
        double Tf3 = Tf2 * Tf;
        xf = ((-4 * Tf * xv0 - 16 * xt + 16 * x0) * t * t * t) / Tf3
           + (( 7 * Tf * xv0 + 24 * xt - 24 * x0) * t * t) / Tf2
           + ((-15 * Tf * xv0 - 36 * xt + 36 * x0) * t) / (4 * Tf)
           +  ( 9 * Tf * xv0 + 16 * xt) / 16;
    }
    if (t > (3 * Tf) / 4) {
        xf = xt;
    }

    // ---- Z Generator ----
    if (t >= 0 && t < Tf / 2) {
        double Tf2 = Tf * Tf;
        double Tf3 = Tf2 * Tf;
        zf = (16 * z0 - 16 * zh) * t * t * t / Tf3
           + (12 * zh - 12 * z0) * t * t / Tf2
           + z0;
    }
    if (t >= Tf / 2) {
        double Tf2 = Tf * Tf;
        zf = (4 * z0 - 4 * zh) * t * t / Tf2
           - (4 * z0 - 4 * zh) * t / Tf
           + z0;
    }

    if (zf <= 0) zf = 0;     // 避免 zf 变负

    return { xf, zf, xf, t };
}

static SupportResult support_curve_generate(double t, double Tf, double x_past,
                                            double t_past, double zf) {
    double average = x_past / (1 - Tf);
    double xf = x_past - average * (t - t_past);
    return { xf, zf };
}

/* ============================================================================
 * 姿态解算 PA_ATTITUDE.cal_ges
 * ========================================================================== */
struct GesResult { double x1, x2, x3, x4, y1, y2, y3, y4; };

static GesResult cal_ges(double PIT, double ROL, double l, double b, double w,
                         double x, double Hc) {
    (void)w;   // 原代码 _y 分量未使用, 仅保留参数对齐
    double YA = 0;
    double P  = deg2rad(PIT);
    double R_ = deg2rad(ROL);
    double Y  = deg2rad(YA);

    double cosP = std::cos(P), sinP = std::sin(P);
    double cosR = std::cos(R_), sinR = std::sin(R_);
    double cosY = std::cos(Y), sinY = std::sin(Y);

    double cy_sr_m = cosY * sinR - cosR * sinP * sinY;
    double sr_sy   = sinR * sinY + cosR * cosY * sinP;

    double ABl_x =  l/2 - x - (l * cosP * cosY)/2 + (b * cosP * sinY)/2;
    double ABl_z = -Hc - (b * cy_sr_m)/2 - (l * sr_sy)/2;
    double AB2_x =  l/2 - x - (l * cosP * cosY)/2 - (b * cosP * sinY)/2;
    double AB2_z =  (b * cy_sr_m)/2 - Hc - (l * sr_sy)/2;
    double AB3_x = (l * cosP * cosY)/2 - x - l/2 + (b * cosP * sinY)/2;
    double AB3_z = (l * sr_sy)/2 - (b * cy_sr_m)/2 - Hc;
    double AB4_x = (l * cosP * cosY)/2 - x - l/2 - (b * cosP * sinY)/2;
    double AB4_z = (b * cy_sr_m)/2 - Hc + (l * sr_sy)/2;

    GesResult r;
    r.x1 = ABl_x;  r.y1 = ABl_z;
    r.x2 = AB2_x;  r.y2 = AB2_z;
    r.x3 = AB4_x;  r.y3 = AB4_z;   // 注意: 原代码 AB4 -> x3/y3
    r.x4 = AB3_x;  r.y4 = AB3_z;   //       AB3 -> x4/y4
    return r;
}

/* ============================================================================
 * 逆运动学 PA_IK.ik (仅 case==0 串联腿)
 * ========================================================================== */
struct IKResult { double ham1, ham2, ham3, ham4, shank1, shank2, shank3, shank4; };

static bool ik_serial(double l1, double l2,
                      double x1, double x2, double x3, double x4,
                      double y1, double y2, double y3, double y4,
                      IKResult& out) {
    auto solve_leg = [&](double x_in, double y, double& ham, double& shank) -> bool {
        double x = -x_in;                 // 原代码 x = -x (镜像)
        double r2 = x * x + y * y;
        if (r2 < 1e-9) return false;
        double r  = std::sqrt(r2);

        double arg_shank = (r2 - l1 * l1 - l2 * l2) / (-2 * l1 * l2);
        if (arg_shank < -1.0 || arg_shank > 1.0) return false;
        shank = PI - std::acos(arg_shank);

        double arg_fai = (l1 * l1 + r2 - l2 * l2) / (2 * l1 * r);
        if (arg_fai < -1.0 || arg_fai > 1.0) return false;
        double fai = std::acos(arg_fai);

        if (x > 0) {
            ham = std::fabs(std::atan(y / x)) - fai;
        } else if (x < 0) {
            ham = PI - std::fabs(std::atan(y / x)) - fai;
        } else {
            ham = PI - 1.5707 - fai;
        }

        shank = rad2deg(shank);
        ham   = rad2deg(ham);
        return true;
    };

    if (!solve_leg(x1, y1, out.ham1,   out.shank1)) return false;
    if (!solve_leg(x2, y2, out.ham2,   out.shank2)) return false;
    if (!solve_leg(x3, y3, out.ham3,   out.shank3)) return false;
    if (!solve_leg(x4, y4, out.ham4,   out.shank4)) return false;
    return true;
}

/* ============================================================================
 * Trot 步态生成 (对应 PA_GAIT.trot)
 * ========================================================================== */
struct GaitResult { double x1, x2, x3, x4, y1, y2, y3, y4; };

static GaitResult trot_gait(double t, double x_target, double z_target,
                            double r1, double r4, double r2, double r3) {
    constexpr double Tf = 0.5;            // Trot 摆动相占空比固定 0.5
    GaitResult g{};

    if (t < Tf) {
        SwingResult   sw = swing_curve_generate(t,         Tf, x_target, z_target, 0, 0, 0);
        SupportResult su = support_curve_generate(0.5 + t, Tf, x_target, 0.5,      0);
        g.x1 = sw.xf * r1;   g.x2 = su.xf * r2;
        g.x3 = sw.xf * r3;   g.x4 = su.xf * r4;
        g.y1 = sw.zf;        g.y2 = su.zf;
        g.y3 = sw.zf;        g.y4 = su.zf;
    } else {
        SwingResult   sw = swing_curve_generate(t - 0.5,   Tf, x_target, z_target, 0, 0, 0);
        SupportResult su = support_curve_generate(t,       Tf, x_target, 0.5,      0);
        g.x1 = su.xf * r1;   g.x2 = sw.xf * r2;
        g.x3 = su.xf * r3;   g.x4 = sw.xf * r4;
        g.y1 = su.zf;        g.y2 = sw.zf;
        g.y3 = su.zf;        g.y4 = sw.zf;
    }
    return g;
}

/* ============================================================================
 * 机构偏置/姿态补偿小工具
 * ========================================================================== */
static inline double mechan_offset_corr(double x) {
    constexpr double p1 = 0.006649;
    constexpr double p2 = 0.4414;
    constexpr double p3 = 5.53;
    return p1 * x * x + p2 * x + p3;
}

static inline double pit_cause_cg_adjust(double sita, double h, double Kp) {
    double v = h * std::tan(sita * Kp);
    return std::round(v * 100.0) / 100.0;
}

/* ============================================================================
 * TrotController 顶层 API 实现
 * ========================================================================== */
void TrotController::move(double spd, double L, double R) {
    state_.spd_goal = spd;
    state_.L = L;
    state_.R = R;
}

void TrotController::height(double goal) { state_.H_goal = goal; }

void TrotController::gesture(double PIT, double ROL, double X) {
    state_.PIT_goal = PIT;
    state_.ROL_goal = ROL;
    state_.X_goal   = X;
}

void TrotController::stable(bool key) {
    if (key && !state_.key_stab) {
        state_.speed_init = cfg.speed;          // 开启自稳时保存当前速度
        cfg.speed = state_.speed_init + 0.01;   // 补偿开陀螺仪造成的速度突变
    } else if (!key && state_.key_stab) {
        cfg.speed = state_.speed_init;          // 关闭自稳时恢复原速度
    }
    state_.key_stab = key;
}

void TrotController::gait(int mode) { state_.gait_mode = mode; }
void TrotController::servoInit(int key) { state_.init_case = key; }
void TrotController::setStopRunNode(bool b) { state_.stop_run_node = b; }

void TrotController::release() {
    if (releaseCb_) releaseCb_();
}

/* ---------- mainloop (对应 padog.mainloop, 仅保留 Trot 分支) ---------- */
void TrotController::mainloop() {
    // 1. 电池/IK 错误检测 -> 决定是否给舵机上电
    bool power_on = (alarm_and_servo_control() == 0);
    if (powerCb_) powerCb_(power_on);
    if (!power_on) {
        if (releaseCb_) releaseCb_();
        return;
    }

    // 2. 锁定屏蔽 (摇晃身体做动作时不跑步态)
    if (state_.stop_run_node) return;

    // 3. 步态模式判断 (仅 Trot)
    if (state_.gait_mode != 0) {
        return;   // 本文件未实现 Walk 等其它步态
    }
    state_.act_tran_mov_kp = cfg.tran_mov_kp;

    // 5. 速度调节器 (P) — 必须先于相位推进，让 spd 先收敛到目标值
    if (state_.spd > state_.spd_goal) {
        state_.spd -= std::fabs(state_.spd - state_.spd_goal) * cfg.Kp_V;
    } else if (state_.spd < state_.spd_goal) {
        state_.spd += std::fabs(state_.spd - state_.spd_goal) * cfg.Kp_V;
    }

    // 3. 步态相位推进 — 使用 spd_goal 判断，避免 spd 瞬态值干扰
    if (state_.t >= 1.0) {
        state_.t = 0;
    } else if (state_.spd_goal == 0 && state_.L == 0 && state_.R == 0) {
        state_.t = 0;
    } else {
        state_.t = state_.t + cfg.speed;
    }

    // 4. Trot 步态足端位置生成
    //    原代码: P_ = PA_GAIT.trot(t, spd*10, h, L, L, R, R)
    //    注意 r1,r2 用 L; r3,r4 用 R (即腿1,2 与 3,4 方向独立)
    //    增大足端位移系数使动作更明显 (原工程 spd 是 0~1 范围)
    GaitResult P = trot_gait(state_.t, state_.spd_goal * 30, cfg.h,
                             state_.L, state_.R, state_.L, state_.R);
    if (state_.spd > state_.spd_goal) {
        state_.spd -= std::fabs(state_.spd - state_.spd_goal) * cfg.Kp_V;
    } else if (state_.spd < state_.spd_goal) {
        state_.spd += std::fabs(state_.spd - state_.spd_goal) * cfg.Kp_V;
    }

    // 6. 高度调节器 (P)
    if (state_.R_H > state_.H_goal) {
        state_.R_H -= std::fabs(state_.R_H - state_.H_goal) * cfg.Kp_H;
    } else if (state_.R_H < state_.H_goal) {
        state_.R_H += std::fabs(state_.R_H - state_.H_goal) * cfg.Kp_H;
    }

    // 7. 姿态调节器 (P) — 自稳时屏蔽
    if (!state_.key_stab) {
        if (state_.PIT_S > state_.PIT_goal) {
            state_.PIT_S -= std::fabs(state_.PIT_S - state_.PIT_goal) * cfg.pit_Kp_G;
        } else if (state_.PIT_S < state_.PIT_goal) {
            state_.PIT_S += std::fabs(state_.PIT_S - state_.PIT_goal) * cfg.pit_Kp_G;
        }
        if (state_.ROL_S > state_.ROL_goal) {
            state_.ROL_S -= std::fabs(state_.ROL_S - state_.ROL_goal) * cfg.rol_Kp_G;
        } else if (state_.ROL_S < state_.ROL_goal) {
            state_.ROL_S += std::fabs(state_.ROL_S - state_.ROL_goal) * cfg.rol_Kp_G;
        }
    } else {
        state_.PIT_S = state_.PIT_goal;
        state_.ROL_S = state_.ROL_goal;
    }

    // 8. X 平移调节器 (P)
    if (state_.X_S > state_.X_goal) {
        state_.X_S -= std::fabs(state_.X_S - state_.X_goal) * state_.act_tran_mov_kp;
    } else if (state_.X_S < state_.X_goal) {
        state_.X_S += std::fabs(state_.X_S - state_.X_goal) * state_.act_tran_mov_kp;
    }

    // 9. 姿态角度限位
    if (state_.PIT_S >=  cfg.pit_max_ang) state_.PIT_S =  cfg.pit_max_ang;
    if (state_.PIT_S <= -cfg.pit_max_ang) state_.PIT_S = -cfg.pit_max_ang;
    if (state_.ROL_S >=  cfg.rol_max_ang) state_.ROL_S =  cfg.rol_max_ang;
    if (state_.ROL_S <= -cfg.rol_max_ang) state_.ROL_S = -cfg.rol_max_ang;

    // 10. Trot 重心补偿
    GesResult P_G;
    if (state_.gait_mode == 0) {
        if (state_.spd >= 0 && (state_.L + state_.R) != 0) {
            P_G = cal_ges(state_.PIT_S, state_.ROL_S, cfg.l, cfg.b, cfg.w,
                          state_.X_S + std::fabs(state_.spd) * cfg.trot_cg_f, state_.R_H);
        } else if (state_.spd < 0 && (state_.L + state_.R) != 0) {
            P_G = cal_ges(state_.PIT_S, state_.ROL_S, cfg.l, cfg.b, cfg.w,
                          state_.X_S - std::fabs(state_.spd) * cfg.trot_cg_b, state_.R_H);
        } else {  // (L+R)==0, 原地旋转
            P_G = cal_ges(state_.PIT_S, state_.ROL_S, cfg.l, cfg.b, cfg.w,
                          state_.X_S + std::fabs(state_.spd) * cfg.trot_cg_t, state_.R_H);
        }
    } else {
        P_G = cal_ges(state_.PIT_S, state_.ROL_S, cfg.l, cfg.b, cfg.w,
                      state_.X_S, state_.R_H);
    }
    state_.ges_x_1 = P_G.x1; state_.ges_x_2 = P_G.x2;
    state_.ges_x_3 = P_G.x3; state_.ges_x_4 = P_G.x4;
    state_.ges_y_1 = P_G.y1; state_.ges_y_2 = P_G.y2;
    state_.ges_y_3 = P_G.y3; state_.ges_y_4 = P_G.y4;

    // 11. 自稳 (静止时启用 IMU 反馈)
    if (state_.key_stab) {
        if (std::fabs(0 - state_.spd) <= 0.1 && state_.L == 0 && state_.R == 0) {
            stab();
        } else {
            state_.PIT_goal = 0;
            state_.ROL_goal = 0;
        }
    }

    // 12. 叠加步态 + 姿态 -> 足端目标
    double fx[4] = {
        P.x1 + P_G.x1,  P.x2 + P_G.x2,  P.x3 + P_G.x3,  P.x4 + P_G.x4
    };
    double fy[4] = {
        P.y1 + P_G.y1,  P.y2 + P_G.y2,  P.y3 + P_G.y3,  P.y4 + P_G.y4
    };

    // 13. 逆运动学
    IKResult A;
    if (!ik_serial(cfg.l1, cfg.l2,
                   fx[0], fx[1], fx[2], fx[3],
                   fy[0], fy[1], fy[2], fy[3], A)) {
        state_.IK_ERROR = 1;
        return;
    }
    state_.IK_ERROR = 0;

    // 14. 输出到舵机
    servo_output(A.ham1, A.ham2, A.ham3, A.ham4,
                 A.shank1, A.shank2, A.shank3, A.shank4);
}

/* ---------- foward_cg_stab (对应 PA_GAIT.foward_cg_stab) ---------- */
void TrotController::foward_cg_stab(double r1, double r4, double r2, double r3,
                                    double gait_need, bool enable) {
    if (!enable) return;
    if ((std::fabs(r1) + std::fabs(r4) + std::fabs(r2) + std::fabs(r3)) == 0) return;

    IMUData d;
    if (!readIMU(d)) return;
    if (!d.calibrated) return;

    double gyro_p = d.pitch_deg;
    state_.X_goal = cfg.in_y + gait_need
                  + pit_cause_cg_adjust((gyro_p - d.pitch_origin) * PI / 180.0, 110, 1.1)
                  + d.gyro_x_radps * 5;
}

/* ---------- stab (对应 PA_STABLIZE.stab) ---------- */
void TrotController::stab() {
    IMUData d;
    if (!readIMU(d)) return;

    // 俯仰 PD
    state_.Sta_Pitch = state_.Sta_Pitch
                     - (d.pitch_deg - d.pitch_origin) * cfg.pit_Kp_G
                     - d.gyro_x_radps * cfg.pit_Kd_G;
    if      (state_.Sta_Pitch >=  cfg.pit_max_ang) state_.Sta_Pitch =  cfg.pit_max_ang;
    else if (state_.Sta_Pitch <= -cfg.pit_max_ang) state_.Sta_Pitch = -cfg.pit_max_ang;

    // 滚转 PD
    state_.Sta_Roll = state_.Sta_Roll
                    - (d.roll_deg - d.roll_origin) * cfg.rol_Kp_G
                    - d.gyro_y_radps * cfg.rol_Kd_G;
    if      (state_.Sta_Roll >=  cfg.rol_max_ang) state_.Sta_Roll =  cfg.rol_max_ang;
    else if (state_.Sta_Roll <= -cfg.rol_max_ang) state_.Sta_Roll = -cfg.rol_max_ang;

    if (d.calibrated) {
        state_.PIT_goal = state_.Sta_Pitch;
        state_.ROL_goal = state_.Sta_Roll;
        state_.acc_z    = d.acc_z_g;
    } else {
        state_.Sta_Pitch = 0;
        state_.Sta_Roll  = 0;
    }
}

/* ---------- alarm_and_servo_control (对应 padog.alarm_and_servo_control) ----------
 * 简化版: 仅保留 IK_ERROR 检测 (电池 ADC 由用户自行在 powerCb 中处理)。
 */
int TrotController::alarm_and_servo_control() {
    int judge_num_node = 0;

    if (state_.IK_ERROR == 1) judge_num_node++;
    if (state_.IK_ERROR == 1 && state_.error_node == 0) {
        state_.error_node  = 1;
        state_.normal_node = 0;
        Serial.println("[Trot] IK error: motion out of range, servos paused");
    }
    if (state_.IK_ERROR == 0 && state_.normal_node == 0) {
        state_.error_node  = 0;
        state_.normal_node = 1;
    }
    return judge_num_node;
}

/* ---------- servo_output (对应 padog.servo_output, 已按用户硬件改写) ----------
 *
 * 用户硬件约定:
 *   - 舵机角度范围 -90~90, 0 为中位
 *   - 度数越大越向前 (大腿: 髋前摆; 小腿: 膝伸直方向)
 *   - 用户舵机库已处理左右镜像, 4 条腿用统一公式
 *   - servos_pin[0..3] = 4 个大腿, servos_pin[4..7] = 4 个小腿
 *
 * 角度推导:
 *   原 Python (PCA9685, 0~180 约定) 对 4 条腿用了两套公式来手动镜像:
 *       腿1,4 大腿: angle = init_h + 90 - ham    (ham 增大 -> angle 减小)
 *       腿2,3 大腿: angle = init_h - 90 + ham    (ham 增大 -> angle 增大)
 *   用户的库已自动处理左右镜像, 因此 4 条腿统一使用同一公式。
 *
 *   方向校验 (见 verify_ham_dir):
 *       足端 x 增大 (向前) -> ham 增大 (约 38° -> 52° -> 69° at y=-110)
 *   用户要求 "度数越大越向前", 因此选 "ham 增大 -> angle 增大" 的公式:
 *       hip_off  = (ham - 90) + init_h_offset
 *   (减 90 是把原 0~180 约定的中位 90 平移到用户 -90~90 约定的中位 0)
 *
 *   小腿方向: shank 增大 = 腿越伸直 (足端离髋越远), 用户希望伸直方向 = "向前":
 *       knee_off = (mechan_offset_corr(shank) - 90) + init_s_offset
 *   (apply_mechan_offset=false 时直接用 shank, 适合线性舵机)
 *
 *   若你的硬件小腿方向相反 (度数越大 = 膝越弯曲), 把 knee_off 取负即可:
 *       knee_off = 90 - mechan_offset_corr(shank) - init_s_offset
 */
void TrotController::servo_output(double ham1, double ham2, double ham3, double ham4,
                                  double shank1, double shank2, double shank3, double shank4) {
    const auto& c = cfg;
    int16_t off[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    // 小腿标定曲线 (原工程硬件非线性标定, 用户线性舵机可关掉)
    auto knee = [&](double shank) -> double {
        if (c.apply_mechan_offset) return mechan_offset_corr(shank);
        return shank;
    };

    if (c.ma_case == 0 && state_.init_case == 0) {
        // 用户舵机顺序: [hip1, hip2, hip3, hip4, knee1, knee2, knee3, knee4]
        // 4 条腿统一公式 (用户库自动镜像)
        double hip_off[4] = {
            ham1 - 90.0 + c.init_1h_offset,
            ham2 - 90.0 + c.init_2h_offset,
            ham3 - 90.0 + c.init_3h_offset,
            ham4 - 90.0 + c.init_4h_offset,
        };
        double knee_off[4] = {
            knee(shank1) - 90.0 + c.init_1s_offset,
            knee(shank2) - 90.0 + c.init_2s_offset,
            knee(shank3) - 90.0 + c.init_3s_offset,
            knee(shank4) - 90.0 + c.init_4s_offset,
        };
        for (int i = 0; i < 4; i++) {
            off[i]     = (int16_t)std::lround(hip_off[i]);
            off[4 + i] = (int16_t)std::lround(knee_off[i]);
        }
    } else {
        // 中位/初始化模式: 所有舵机回到中位 (off = init_*_offset, 默认 0)
        double hip_off[4] = {
            (double)c.init_1h_offset, (double)c.init_2h_offset,
            (double)c.init_3h_offset, (double)c.init_4h_offset,
        };
        double knee_off[4] = {
            (double)c.init_1s_offset, (double)c.init_2s_offset,
            (double)c.init_3s_offset, (double)c.init_4s_offset,
        };
        for (int i = 0; i < 4; i++) {
            off[i]     = (int16_t)std::lround(hip_off[i]);
            off[4 + i] = (int16_t)std::lround(knee_off[i]);
        }
    }

    // 调用用户舵机库批量下发: {hip1, hip2, hip3, hip4, knee1, knee2, knee3, knee4}
    m_servo::set_angle_90_multi({ off[0], off[1], off[2], off[3],
                                  off[4], off[5], off[6], off[7] });
}

/* ============================================================================
 * FreeRTOS mainloop 任务
 * ========================================================================== */
static TaskHandle_t s_trot_task_handle = nullptr;
static bool          s_trot_task_run    = false;

static void trot_task(void* /*arg*/) {
    // 5ms 周期 (与原 ESP32 MicroPython Timer period=5 一致)
    const TickType_t period = pdMS_TO_TICKS(5);
    TickType_t last = xTaskGetTickCount();
    while (s_trot_task_run) {
        trot.mainloop();
        vTaskDelayUntil(&last, period);
    }
    s_trot_task_handle = nullptr;
    vTaskDelete(nullptr);
}

void trotStartTask(uint8_t priority, uint32_t stackSize) {
    if (s_trot_task_handle) return;     // 已启动
    s_trot_task_run = true;
    xTaskCreate(
        trot_task,
        "trot",
        stackSize,
        nullptr,
        priority,
        &s_trot_task_handle
    );
}

void trotStopTask() {
    s_trot_task_run = false;
    // 任务会在下次循环退出并自删除
}

} // namespace padynamics

/* ============================================================================
 * IMU 读取 weak 默认实现 (用户在自己的 .ino 中重新定义即可覆盖)
 * ========================================================================== */
bool __attribute__((weak)) readIMU(padynamics::IMUData& out) {
    (void)out;
    return false;   // 默认无 IMU, 自稳不生效
}
