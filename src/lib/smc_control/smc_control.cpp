/**
 * @file smc_control.cpp
 */
#include "smc_control.hpp"

#include <px4_platform_common/defines.h>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

void SMC_control::setGains(const matrix::Vector3f &lambda, const matrix::Vector3f &K,  const float yaw_weight)
{
	_gain_lambda = -lambda;
	_gain_K = K;
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	// Set Moment of Inertia
	_moi(0,0) = 1; // Ixx
	_moi(0,1) = 0; // Ixy
	_moi(0,2) = 0; // Ixz

	_moi(1,0) = 0; // Ixy
	_moi(1,1) = 1; // Iyy
	_moi(1,2) = 0; // Iyz

	_moi(2,0) = 0; // Ixz
	_moi(2,1) = 0; // Iyz
	_moi(2,2) = 1; // Izz

	// compensate for the effect of the yaw weight rescaling the output
	if (_yaw_w > 1e-4f) {
		_gain_K(2) /= _yaw_w;
		_gain_lambda(2) /= _yaw_w;
	}
}

matrix::Vector3f SMC_control::saturation(const matrix::Vector3f &s)
{
	Vector3f output;
	for (int i=0;i<3;i++){
		output(i) = math::constrain(s(i), -1.f, 1.f);
	}
	return output;
}

matrix::Vector3f SMC_control::signum(const matrix::Vector3f &s)
{
	Vector3f output;
	for (int i=0;i<3;i++){
		output(i) = (s(i) < 0 ? -1:1);
	}
	return output;
}

matrix::Vector3f SMC_control::update(const matrix::Quatf q, const matrix::Vector3f &rate, const bool landed)
{
	Quatf qd = _attitude_setpoint_q;

	// calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
	const Vector3f e_z = q.dcm_z();
	const Vector3f e_z_d = qd.dcm_z();
	Quatf qd_red(e_z, e_z_d);

	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
		// In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
		// full attitude control anyways generates no yaw input and directly takes the combination of
		// roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
		qd_red = qd;

	} else {
		// transform rotation from current to desired thrust vector into a world frame reduced desired attitude
		qd_red *= q;
	}

	// mix full and reduced desired attitude
	Quatf q_mix = qd_red.inversed() * qd;
	q_mix.canonicalize();
	// catch numerical problems with the domain of acosf and asinf
	q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);
	q_mix(3) = math::constrain(q_mix(3), -1.f, 1.f);
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(q_mix(0))), 0, 0, sinf(_yaw_w * asinf(q_mix(3))));

	// quaternion attitude control law, qe is rotation from q to qd
	const Quatf qe = q.inversed() * qd;
	const Quatf eq = qe.canonical();
	const Quatf eq_dot = -.5f * (Quatf(0 ,rate(0), rate(1), rate(2)) * eq);
	const Vector3f L = (eq.imag().cross(_moi*rate)+_moi*eq_dot.imag()).emult(_gain_lambda);

	// Set sliding surface
	const Vector3f s = rate+qe.imag().emult(_gain_lambda);

	// Vector3f torque = -L-saturation(s).emult(_gain_K);
	Vector3f torque = -L-signum(s).emult(_gain_K);
	return torque;
}

void SMC_control::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = 0;
	rate_ctrl_status.pitchspeed_integ = 0;
	rate_ctrl_status.yawspeed_integ = 0;
}
