#include "foc.h"
#include "arm_math.h"
#include "motor_runtime_param.h"
#include <stdbool.h>
#define deg2rad(a) (PI * (a) / 180)
#define rad2deg(a) (180 * (a) / PI)
#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define rad60 deg2rad(60)
#define SQRT3 1.73205080756887729353

arm_pid_instance_f32 pid_position;
arm_pid_instance_f32 pid_speed;
arm_pid_instance_f32 pid_torque_d;
arm_pid_instance_f32 pid_torque_q;
motor_control_context_t motor_control_context;

static void svpwm(float phi, float d, float q, float *d_u, float *d_v, float *d_w)
{
    d = min(d, 1);
    d = max(d, -1);
    q = min(q, 1);
    q = max(q, -1);
    const int v[6][3] = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}, {1, 0, 1}};
    const int K_to_sector[] = {4, 6, 5, 5, 3, 1, 2, 2};
    float sin_phi = arm_sin_f32(phi);
    float cos_phi = arm_cos_f32(phi);
    float alpha = 0;
    float beta = 0;
    arm_inv_park_f32(d, q, &alpha, &beta, sin_phi, cos_phi);

    bool A = beta > 0;
    bool B = fabs(beta) > SQRT3 * fabs(alpha);
    bool C = alpha > 0;

    int K = 4 * A + 2 * B + C;
    int sector = K_to_sector[K];

    float t_m = arm_sin_f32(sector * rad60) * alpha - arm_cos_f32(sector * rad60) * beta;
    float t_n = beta * arm_cos_f32(sector * rad60 - rad60) - alpha * arm_sin_f32(sector * rad60 - rad60);
    float t_0 = 1 - t_m - t_n;

    *d_u = t_m * v[sector - 1][0] + t_n * v[(sector) % 6][0] + t_0 / 2;
    *d_v = t_m * v[sector - 1][1] + t_n * v[(sector) % 6][1] + t_0 / 2;
    *d_w = t_m * v[sector - 1][2] + t_n * v[(sector) % 6][2] + t_0 / 2;
}

__attribute__((weak)) void set_pwm_duty(float d_u, float d_v, float d_w)
{
    while (1)
        ;
}

void foc_forward(float d, float q, float rotor_rad)
{
    float d_u = 0;
    float d_v = 0;
    float d_w = 0;
    svpwm(rotor_rad, d, q, &d_u, &d_v, &d_w);
    set_pwm_duty(d_u, d_v, d_w);
}

static float position_loop(float rad)
{
    float diff = cycle_diff(rad - motor_logic_angle, position_cycle);
    return arm_pid_f32(&pid_position, diff);
}

static float speed_loop(float speed_rad)
{
    float diff = speed_rad - motor_speed;
    return arm_pid_f32(&pid_speed, diff);
}

static float torque_d_loop(float d)
{
    float diff = d - motor_i_d / MAX_CURRENT;
    float out_unlimited = arm_pid_f32(&pid_torque_d, diff);
    float out_limited = 0;
    out_limited = min(out_unlimited, 1);
    out_limited = max(out_limited, -1);

    float error_integral_windup = out_limited - out_unlimited; //  积分饱和误差
    float Kt = 0.7f;                                           //  后积分增益 (Anti-windup Gain)
    pid_torque_d.state[0] -= (pid_torque_d.Ki * Kt * error_integral_windup);

    return out_limited;
}

static float torque_q_loop(float q)
{
    float diff = q - motor_i_q / MAX_CURRENT;
    float out_unlimited = arm_pid_f32(&pid_torque_q, diff);
    float out_limited = 0;
    out_limited = min(out_unlimited, 1);
    out_limited = max(out_limited, -1);

    float Kt = 0.7f;                                           //  后积分增益 (Anti-windup Gain)
    float error_integral_windup = out_limited - out_unlimited; //  积分饱和误差
    pid_torque_q.state[0] -= (pid_torque_q.Ki * Kt * error_integral_windup);

    return out_unlimited;
}

void lib_position_control(float rad)
{
    float d = 0;
    float q = position_loop(rad);
    foc_forward(d, q, rotor_logic_angle);
}

void lib_speed_control(float speed)
{
    float d = 0;
    float q = speed_loop(speed);
    foc_forward(d, q, rotor_logic_angle);
}

void lib_torque_control(float torque_norm_d, float torque_norm_q)
{
    float d = torque_d_loop(torque_norm_d);
    float q = torque_q_loop(torque_norm_q);
    foc_forward(d, q, rotor_logic_angle);
}

void lib_speed_torque_control(float speed_rad, float max_torque_norm)
{
    float torque_norm = speed_loop(speed_rad);
    torque_norm = min(fabs(torque_norm), max_torque_norm) * (torque_norm > 0 ? 1 : -1);
    lib_torque_control(0, torque_norm);
}

void lib_position_speed_torque_control(float position_rad, float max_speed_rad, float max_torque_norm)
{
    float speed_rad = position_loop(position_rad);
    speed_rad = min(fabs(speed_rad), max_speed_rad) * (speed_rad > 0 ? 1 : -1);
    lib_speed_torque_control(speed_rad, max_torque_norm);
}

void set_motor_pid(
    float position_p, float position_i, float position_d,
    float speed_p, float speed_i, float speed_d,
    float torque_d_p, float torque_d_i, float torque_d_d,
    float torque_q_p, float torque_q_i, float torque_q_d)
{
    pid_position.Kp = position_p;
    pid_position.Ki = position_i;
    pid_position.Kd = position_d;

    pid_speed.Kp = speed_p;
    pid_speed.Ki = speed_i;
    pid_speed.Kd = speed_d;

    pid_torque_d.Kp = torque_d_p;
    pid_torque_d.Ki = torque_d_i;
    pid_torque_d.Kd = torque_d_d;

    pid_torque_q.Kp = torque_q_p;
    pid_torque_q.Ki = torque_q_i;
    pid_torque_q.Kd = torque_q_d;
    arm_pid_init_f32(&pid_position, false);
    arm_pid_init_f32(&pid_speed, false);
    arm_pid_init_f32(&pid_torque_d, false);
    arm_pid_init_f32(&pid_torque_q, false);
}
/*
输入：
    diff：原始差值（如角度差、位置差）。
    cycle：信号周期（如电机电角度周期 2π、机械位置周期等）。
输出：
    调整后的差值，范围为 [-cycle/2, cycle/2]，即最短路径差值。
    编码器角度位于0 到2 π 之间，实际上从359度到1度实际只旋转了2度，但是差值是358度，因此计算差值的时候要把差值转换到− π 到π 之间。
*/
float cycle_diff(float diff, float cycle)
{
    if (diff > (cycle / 2))       // 差值超过正半周期
        diff -= cycle;            // 减去一个周期（等效于取负数）
    else if (diff < (-cycle / 2)) // 差值小于负半周期
        diff += cycle;            // 加上一个周期（等效于取正数）
    return diff;
}