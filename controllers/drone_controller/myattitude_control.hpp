#pragma once

#include <array>
#include <cmath>
#include <algorithm>





class Quaternion {
public:
    float w, x, y, z;

    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
    Quaternion(const std::array<float, 3> &src, const std::array<float, 3> &dst, float eps = 1e-5f) 
    {
        // 计算叉积（旋转轴）
        std::array<float, 3> cr = {
            src[1] * dst[2] - src[2] * dst[1],
            src[2] * dst[0] - src[0] * dst[2],
            src[0] * dst[1] - src[1] * dst[0]
        };
        // 计算点积（旋转角的余弦）
        float dt = src[0] * dst[0] + src[1] * dst[1] + src[2] * dst[2];

        // 判断特殊情况
        if (std::sqrt(cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]) < eps && dt < 0) {
            // 处理源和目标向量反向的情况
            // 选择一个垂直于源向量的基准方向
            if (std::fabs(src[0]) < std::fabs(src[1]) && std::fabs(src[0]) < std::fabs(src[2])) {
                cr = {1.0f, 0.0f, 0.0f};  // 选择x轴
            } else if (std::fabs(src[1]) < std::fabs(src[2])) {
                cr = {0.0f, 1.0f, 0.0f};  // 选择y轴
            } else {
                cr = {0.0f, 0.0f, 1.0f};  // 选择z轴
            }

            // 计算旋转轴
            cr = {src[1] * cr[2] - src[2] * cr[1], src[2] * cr[0] - src[0] * cr[2], src[0] * cr[1] - src[1] * cr[0]};

            w = 0;  // 180度旋转，实部为零
        } else {
            // 一般情况，计算四元数
            w = dt + std::sqrt(src[0] * src[0] + src[1] * src[1] + src[2] * src[2]) * std::sqrt(dst[0] * dst[0] + dst[1] * dst[1] + dst[2] * dst[2]);
        }

        // 将叉积结果赋给四元数的虚部
        x = cr[0];
        y = cr[1];
        z = cr[2];
        // 归一化四元数
        float norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (norm > 0) {
            w /= norm;
            x /= norm;
            y /= norm;
            z /= norm;
        }        
    }
    // Quaternion multiplication
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }

    // Quaternion inverse
    Quaternion inversed() const {
        float norm = std::sqrt(w * w + x * x + y * y + z * z);
        return Quaternion(w/norm, -x/norm, -y/norm, -z/norm);
    }

    // Quaternion from axis-angle
    static Quaternion fromAxisAngle(const std::array<float, 3>& axis, float angle) {
        float half_angle = angle * 0.5f;
        float s = std::sin(half_angle);
        return Quaternion(std::cos(half_angle), axis[0] * s, axis[1] * s, axis[2] * s);
    }



    // Normalize quaternion
    void normalize() {
        float norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (norm > 0) {
            w /= norm;
            x /= norm;
            y /= norm;
            z /= norm;
        }
    }

    // Canonicalize quaternion
    Quaternion canonical() const {
        return w < 0 ? Quaternion(-w, -x, -y, -z) : *this;
    }

    // Imaginary part
    std::array<float, 3> imag() const {
        return {x, y, z};
    }

    // Convert to DCM z-axis
    std::array<float, 3> dcm_z() const {
        return {
            2 * (w * y + x * z),
            2 * (y * z - w * x),
            w*w - x*x - y*y + z*z
            
        };
    }
};



class AttitudeControl {
public:
    void setProportionalGain(const std::array<float, 3>& proportional_gain, float yaw_weight);
    std::array<float, 3> update(const Quaternion& q);

    // Setter for attitude setpoint
    void setAttitudeSetpoint(const Quaternion& qd) { _attitude_setpoint_q = qd; }

    // Getter for attitude setpoint
    Quaternion getAttitudeSetpoint() const { return _attitude_setpoint_q; }

private:
    Quaternion _attitude_setpoint_q;
    std::array<float, 3> _proportional_gain = {1.0f, 1.0f, 1.0f};
    float _yaw_w = 1.0f;
    float _yawspeed_setpoint = 0.0f;
    std::array<float, 3> _rate_limit = {100.0f, 100.0f, 100.0f};

    float constrain(float val, float min_val, float max_val) const {
        return std::max(min_val, std::min(val, max_val));
    }
};


