#include <limits>

#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>

#include "ros1_maze_escape/velocity_adapter.hpp"

namespace ros1_maze_escape {
namespace {

VelocityAdapter makeDefaultAdapter() {
  return VelocityAdapter(VelocityLimits{});
}

void expectStopped(const puppy_control::Velocity& command) {
  EXPECT_FLOAT_EQ(command.x, 0.0F);
  EXPECT_FLOAT_EQ(command.y, 0.0F);
  EXPECT_FLOAT_EQ(command.yaw_rate, 0.0F);
}

TEST(VelocityAdapterTest, ConvertsMetersPerSecondToCentimetersPerSecondOnce) {
  const VelocityAdapter adapter = makeDefaultAdapter();
  geometry_msgs::Twist input;
  input.linear.x = 0.05;

  const VelocityConversion result = adapter.convert(input, false);

  ASSERT_TRUE(result.input_valid);
  EXPECT_FLOAT_EQ(result.command.x, 5.0F);
}

TEST(VelocityAdapterTest, ClampsForwardSpeed) {
  const VelocityAdapter adapter = makeDefaultAdapter();
  geometry_msgs::Twist input;
  input.linear.x = 0.20;

  const VelocityConversion result = adapter.convert(input, false);

  ASSERT_TRUE(result.input_valid);
  EXPECT_FLOAT_EQ(result.command.x, 10.0F);
}

TEST(VelocityAdapterTest, ClampsReverseSpeedToConfiguredLimit) {
  VelocityLimits limits;
  limits.max_reverse_cm_s = 6.0;
  const VelocityAdapter adapter(limits);
  geometry_msgs::Twist input;
  input.linear.x = -0.20;

  const VelocityConversion result = adapter.convert(input, false);

  ASSERT_TRUE(result.input_valid);
  EXPECT_FLOAT_EQ(result.command.x, -6.0F);
}

TEST(VelocityAdapterTest, ClampsYawRate) {
  const VelocityAdapter adapter = makeDefaultAdapter();
  geometry_msgs::Twist input;
  input.angular.z = 1.0;

  const VelocityConversion result = adapter.convert(input, false);

  ASSERT_TRUE(result.input_valid);
  EXPECT_FLOAT_EQ(result.command.yaw_rate, 0.28F);
}

TEST(VelocityAdapterTest, AlwaysForcesLateralVelocityToZero) {
  const VelocityAdapter adapter = makeDefaultAdapter();
  geometry_msgs::Twist input;
  input.linear.x = 0.05;
  input.linear.y = 3.0;

  const VelocityConversion result = adapter.convert(input, false);

  ASSERT_TRUE(result.input_valid);
  EXPECT_FLOAT_EQ(result.command.x, 5.0F);
  EXPECT_FLOAT_EQ(result.command.y, 0.0F);
}

TEST(VelocityAdapterTest, EmergencyStopForcesEveryOutputToZero) {
  const VelocityAdapter adapter = makeDefaultAdapter();
  geometry_msgs::Twist input;
  input.linear.x = 0.05;
  input.linear.y = 1.0;
  input.angular.z = 0.10;

  const VelocityConversion result = adapter.convert(input, true);

  ASSERT_TRUE(result.input_valid);
  expectStopped(result.command);
}

TEST(VelocityAdapterTest, NanInputIsInvalidAndStops) {
  const VelocityAdapter adapter = makeDefaultAdapter();
  geometry_msgs::Twist input;
  input.linear.x = std::numeric_limits<double>::quiet_NaN();

  const VelocityConversion result = adapter.convert(input, false);

  EXPECT_FALSE(result.input_valid);
  expectStopped(result.command);
}

TEST(VelocityAdapterTest, InfiniteInputIsInvalidAndStops) {
  const VelocityAdapter adapter = makeDefaultAdapter();
  geometry_msgs::Twist input;
  input.angular.z = std::numeric_limits<double>::infinity();

  const VelocityConversion result = adapter.convert(input, false);

  EXPECT_FALSE(result.input_valid);
  expectStopped(result.command);
}

}  // namespace
}  // namespace ros1_maze_escape

