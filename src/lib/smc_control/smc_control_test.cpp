#include <gtest/gtest.h>
#include <lib/smc_control/smc_control.hpp>

using namespace matrix;

TEST(SMCControlTest, AllZeroCase)
{
	SMC_control smc_control;
	Vector3f torque = smc_control.update(Quatf(), Vector3f(), false);
	EXPECT_EQ(torque, Vector3f());
}
