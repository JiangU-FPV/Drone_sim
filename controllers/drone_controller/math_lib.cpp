#include "math_lib.hpp"


float linear_scale(float value, float in_min, float in_max, float out_min, float out_max) 
{
    // 防止输入范围无效
    if (in_max == in_min) {
        return out_min; // 或者返回错误值，例如 NAN
    }
    // 线性映射公式
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min);
}

float dead_zone(float value,float dead)
{
  if(value<dead&&value>-dead)
  {
    return 0;
  }
  return value;
}

/**
 * @brief 将摇杆输入映射到无人机倾斜方向向量
 * @param pitch_stick 俯仰摇杆输入值，范围[-1,1]
 * @param roll_stick 横滚摇杆输入值，范围[-1,1]
 * @param max_tilt_angle 最大倾斜角度限制（弧度）
 * @param direction_vector 输出的方向向量（3维）
 */
void stick_to_direction_vector(float pitch_stick, float roll_stick, float max_tilt_angle, float (&direction_vector)[3]) {
    // 确保输入在[-1,1]范围内
    pitch_stick = m_constrain(pitch_stick, -1.0f, 1.0f);
    roll_stick = m_constrain(roll_stick, -1.0f, 1.0f);
    
    // 计算总倾斜量（考虑两个摇杆的叠加效应）
    float total_tilt = sqrtf(pitch_stick * pitch_stick + roll_stick * roll_stick);
    total_tilt = m_constrain(total_tilt, 0.0f, 1.0f);
    
    // 计算实际倾斜角度（考虑最大角度限制）
    float tilt_angle = total_tilt * max_tilt_angle;
    
    // 计算方向向量
    float sin_tilt = sinf(tilt_angle);
    float cos_tilt = cosf(tilt_angle);
    
    // 如果总倾斜量为0，直接返回垂直向上的向量
    if (total_tilt < 1e-6f) {
        direction_vector[0] = 0.0f;
        direction_vector[1] = 0.0f;
        direction_vector[2] = 1.0f;
        return;
    }
    
    // 计算方向向量
    direction_vector[0] = (roll_stick / total_tilt) * sin_tilt;  // x分量
    direction_vector[1] = (pitch_stick / total_tilt) * sin_tilt; // y分量
    direction_vector[2] = cos_tilt;                             // z分量
}

void stick_to_quaternion(float pitch_stick, float roll_stick, float yaw_angle, float max_tilt_angle, float (&target_quat)[4]) {
    // 确保输入在[-1,1]范围内
    pitch_stick = m_constrain(pitch_stick, -1.0f, 1.0f);
    roll_stick = m_constrain(roll_stick, -1.0f, 1.0f);
    
    // 计算总倾斜量（考虑两个摇杆的叠加效应）
    float total_tilt = sqrtf(pitch_stick * pitch_stick + roll_stick * roll_stick);
    total_tilt = m_constrain(total_tilt, 0.0f, 1.0f);
    
    // 如果总倾斜量为0，只考虑偏航
    if (total_tilt < 1e-6f) {
        float half_yaw = yaw_angle * 0.5f;
        target_quat[0] = cosf(half_yaw);  // w
        target_quat[1] = 0.0f;            // x
        target_quat[2] = 0.0f;            // y
        target_quat[3] = sinf(half_yaw);  // z
        return;
    }
    
    // 计算实际倾斜角度（考虑最大角度限制）
    float tilt_angle = total_tilt * max_tilt_angle;
    
    // 计算旋转轴（归一化）
    float axis_y = -roll_stick / total_tilt;
    float axis_x = -pitch_stick / total_tilt;
    float axis_z = 0.0f;
    
    // 计算倾斜四元数
    float half_tilt = tilt_angle * 0.5f;
    float sin_half_tilt = sinf(half_tilt);
    float cos_half_tilt = cosf(half_tilt);
    
    // 计算偏航四元数
    float half_yaw = yaw_angle * 0.5f;
    float sin_half_yaw = sinf(half_yaw);
    float cos_half_yaw = cosf(half_yaw);
    
    // 组合四元数（先倾斜后偏航）
    target_quat[0] = cos_half_tilt * cos_half_yaw;                    // w
    target_quat[1] = axis_x * sin_half_tilt * cos_half_yaw;          // x
    target_quat[2] = axis_y * sin_half_tilt * cos_half_yaw;          // y
    target_quat[3] = cos_half_tilt * sin_half_yaw;                   // z
}

void euler_to_quaternion(float pitch, float roll, float yaw, float (&quat)[4]) {
    // 计算每个角度的半角
    float half_roll = -pitch * 0.5f;
    float half_pitch = roll * 0.5f;
    float half_yaw = yaw * 0.5f;

    // 计算每个角度的正弦和余弦
    float sin_roll = sinf(half_roll);
    float cos_roll = cosf(half_roll);
    float sin_pitch = sinf(half_pitch);
    float cos_pitch = cosf(half_pitch);
    float sin_yaw = sinf(half_yaw);
    float cos_yaw = cosf(half_yaw);

    // 计算四元数的各个分量
    quat[0] = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;  // w
    quat[1] = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;  // x
    quat[2] = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;  // y
    quat[3] = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;  // z
}

