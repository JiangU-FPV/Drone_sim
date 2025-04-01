#include "myattitude_control.hpp"



void AttitudeControl::setProportionalGain(const std::array<float, 3>& proportional_gain, float yaw_weight) {
    _proportional_gain = proportional_gain;
    _yaw_w = constrain(yaw_weight, 0.0f, 1.0f);

    // Compensate for yaw weight
    if (_yaw_w > 1e-4f) {
        _proportional_gain[2] /= _yaw_w;
    }
}

std::array<float, 3> AttitudeControl::update(const Quaternion& q) {
    Quaternion qd = _attitude_setpoint_q;

    // Calculate reduced desired attitude neglecting yaw
    auto e_z = q.dcm_z();
    auto e_z_d = qd.dcm_z();
    // Quaternion qd_red = Quaternion::fromAxisAngle(e_z, std::acos(std::min(std::max(e_z[0] * e_z_d[0] +
    //                                                                               e_z[1] * e_z_d[1] +
    //                                                                               e_z[2] * e_z_d[2], -1.0f), 1.0f)));

    Quaternion qd_red(e_z,e_z_d);         

    if (std::fabs(qd_red.x) > (1.0f - 1e-5f) || std::fabs(qd_red.y) > (1.0f - 1e-5f)) {
        qd_red = qd; // Special case handling
    } else {
        qd_red = qd_red * q;
    }

    // Mix full and reduced desired attitude
    Quaternion q_mix = qd_red.inversed() * qd;
    q_mix.w = constrain(q_mix.w, -1.0f, 1.0f);
    q_mix.z = constrain(q_mix.z, -1.0f, 1.0f);
    qd = qd_red * Quaternion(std::cos(_yaw_w * std::acos(q_mix.w)), 0, 0, std::sin(_yaw_w * std::asin(q_mix.z)));

    // Quaternion attitude control law
    Quaternion qe = q.inversed() * qd;
    auto eq = qe.canonical().imag();
    for (auto& val : eq) val *= 2.0f;

    // Calculate angular rate setpoint
    std::array<float, 3> rate_setpoint;
    for (int i = 0; i < 3; ++i) {
        rate_setpoint[i] = eq[i] * _proportional_gain[i];
    }

    // Feed forward yaw setpoint rate
    if (std::isfinite(_yawspeed_setpoint)) {
        auto world_z = q.inversed().dcm_z();
        for (int i = 0; i < 3; ++i) {
            rate_setpoint[i] += world_z[i] * _yawspeed_setpoint;
        }
    }

    // Limit rates
    for (int i = 0; i < 3; ++i) {
        rate_setpoint[i] = constrain(rate_setpoint[i], -_rate_limit[i], _rate_limit[i]);
    }

    return rate_setpoint;
}
