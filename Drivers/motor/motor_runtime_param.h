#pragma once
#include "conf.h"
#define rotor_phy_angle (motor_logic_angle - rotor_zero_angle) // 转子物理角度
#define rotor_logic_angle rotor_phy_angle *POLE_PAIRS          // 转子多圈角度
extern float motor_i_u;
extern float motor_i_v;
extern float motor_i_d;
extern float motor_i_q;
extern float motor_speed;
extern float motor_logic_angle; // 电机多圈角度
extern float encoder_angle;     // 编码器直接读出的角度
extern float rotor_zero_angle;  // 转子d轴与线圈d轴重合时的编码器角度
