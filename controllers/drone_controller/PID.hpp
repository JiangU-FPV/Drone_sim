#pragma once

#include <cmath>

class PID
{
public:
	PID() = default;
	virtual ~PID() = default;
	void setOutputLimit(const float limit) { _limit_output = limit; }
	void setIntegralLimit(const float limit) { _limit_integral = limit; }
	void setGains(const float P, const float I, const float D);
	void setSetpoint(const float setpoint) { _setpoint = setpoint; }
	float update(const float feedback, const float dt, const bool update_integral = true);
	float getIntegral() { return _integral; }
	void resetIntegral() { _integral = 0.f; };
	void resetDerivative() { _last_feedback = NAN; };
private:
	void updateIntegral(float error, const float dt);
	float updateDerivative(float feedback, const float dt);

	float _setpoint{0.f}; ///< current setpoint to track
	float _integral{0.f}; ///< integral state
	float _last_feedback{NAN};

	// Gains, Limits
	float _gain_proportional{0.f};
	float _gain_integral{0.f};
	float _gain_derivative{0.f};
	float _limit_integral{0.f};
	float _limit_output{0.f};
};