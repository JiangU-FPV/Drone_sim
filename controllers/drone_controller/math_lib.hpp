#pragma once
#include <cmath>
#define m_abs(x) 	((x)>0? (x):(-(x)))
#define m_constrain(x, min, max)	((x>max)?max:(x<min?min:x))

float linear_scale(float value, float in_min, float in_max, float out_min, float out_max);
float dead_zone(float value,float dead);

/**
 * @brief 将摇杆输入映射到无人机目标姿态四元数
 * @param pitch_stick 俯仰摇杆输入值，范围[-1,1]
 * @param roll_stick 横滚摇杆输入值，范围[-1,1]
 * @param yaw_angle 偏航角期望值，范围[-π, π]
 * @param max_tilt_angle 最大倾斜角度限制（弧度制）
 * @param target_quat 输出的目标姿态四元数，会被函数修改
 */
void stick_to_quaternion(float pitch_stick, float roll_stick, float yaw_angle, float max_tilt_angle, float (&target_quat)[4]);
void stick_to_direction_vector(float pitch_stick, float roll_stick, float max_tilt_angle, float (&direction_vector)[3]);

/**
 * @brief 将欧拉角转换为四元数
 * @param roll 横滚角（弧度）
 * @param pitch 俯仰角（弧度）
 * @param yaw 偏航角（弧度）
 * @param quat 输出的四元数，格式为[w,x,y,z]
 */
void euler_to_quaternion(float roll, float pitch, float yaw, float (&quat)[4]);
