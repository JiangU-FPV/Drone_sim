#include "PID.hpp"
#include "math_lib.hpp"
#include <cfloat>

void PID::setGains(const float P, const float I, const float D)
{
	_gain_proportional = P;
	_gain_integral = I;
	_gain_derivative = D;
}

float PID::update(const float feedback, const float dt, const bool update_integral)
{
	const float error = _setpoint - feedback;
	const float output = (_gain_proportional * error) + _integral + (_gain_derivative * updateDerivative(feedback, dt));

	if (update_integral) {
		updateIntegral(error, dt);
	}

	_last_feedback = feedback;
	return m_constrain(output, -_limit_output, _limit_output);
}

void PID::updateIntegral(float error, const float dt)
{
	const float integral_new = _integral + _gain_integral * error * dt;

	if (std::isfinite(integral_new)) {
		_integral = m_constrain(integral_new, -_limit_integral, _limit_integral);
	}
}

float PID::updateDerivative(float feedback, const float dt)
{
	float feedback_change = 0.f;

	if ((dt > FLT_EPSILON) && std::isfinite(_last_feedback)) {
		feedback_change = (feedback - _last_feedback) / dt;
	}

	return feedback_change;
}