/**
 * @file smc_control.hpp
 *
 * Sliding mode controller based on Quaternion & angular velocity
 */

#pragma once

#include <matrix/matrix/math.hpp>

#include <mathlib/mathlib.h>
#include <uORB/topics/rate_ctrl_status.h>

class SMC_control
{
public:
	SMC_control()=default;
	~SMC_control()=default;
	/**
	 * Set parameter for Sliding mode control
	 * @param lambda 3x1 vector : Sliding surface convergence
	 * @param K 3x1 vector : Sliding surface gain
	*/
	void setGains(const matrix::Vector3f &lambda, const matrix::Vector3f &K);

	/**
	 * Set a new attitude setpoint replacing the one tracked before
	 * @param qd desired vehicle attitude setpoint
	 * @param yawspeed_setpoint [rad/s] yaw feed forward angular rate in world frame
	 */
	void setAttitudeSetpoint(const matrix::Quatf &qd)
	{
		_attitude_setpoint_q = qd;
		_attitude_setpoint_q.normalize();
	}

	/**
	 * Adjust last known attitude setpoint by a delta rotation
	 * Optional use to avoid glitches when attitude estimate reference e.g. heading changes.
	 * @param q_delta delta rotation to apply
	 */
	void adaptAttitudeSetpoint(const matrix::Quatf &q_delta)
	{
		_attitude_setpoint_q = q_delta * _attitude_setpoint_q;
		_attitude_setpoint_q.normalize();
	}

	/**
	 * Get status message of controller for logging/debugging
	 * @param rate_ctrl_status status message to fill with internal states
	 */
	void getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status);

	/**
	 * Run saturation function for sliding mode control
	 * @param s sliding surface
	 * @return [-1,1] saturation output
	 *
	*/
	matrix::Vector3f saturation(const matrix::Vector3f &s);

	/**
	 * Run one control loop cycle calculation
	 * @param q estimation of the current vehicle attitude unit quaternion
	 * @param rate estimation of the current vehicle angular rate
	 * @param dt desired vehicle angular rate setpoint
	 * @return [-1,1] normalized torque vector to apply to the vehicle
	 */
	matrix::Vector3f update(const matrix::Quatf q, const matrix::Vector3f &rate, const bool landed);


private:
	matrix::Vector3f _gain_lambda;
	matrix::Vector3f _gain_K;
	matrix::Matrix<float, 3,3> _moi;
	matrix::Quatf _attitude_setpoint_q; ///< latest known attitude setpoint e.g. from position control
};
