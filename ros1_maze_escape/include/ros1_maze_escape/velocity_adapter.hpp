#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <geometry_msgs/Twist.h>
#include <puppy_control/Velocity.h>

namespace ros1_maze_escape {

struct VelocityLimits {
  double max_forward_cm_s{10.0};
  double max_reverse_cm_s{10.0};
  double max_yaw_rate_rad_s{0.28};
};

struct VelocityConversion {
  puppy_control::Velocity command;
  bool input_valid{false};
};

class VelocityAdapter {
 public:
  explicit VelocityAdapter(const VelocityLimits& limits) : limits_(limits) {
    if (!std::isfinite(limits_.max_forward_cm_s) ||
        !std::isfinite(limits_.max_reverse_cm_s) ||
        !std::isfinite(limits_.max_yaw_rate_rad_s) ||
        limits_.max_forward_cm_s < 0.0 ||
        limits_.max_reverse_cm_s < 0.0 ||
        limits_.max_yaw_rate_rad_s < 0.0) {
      throw std::invalid_argument("velocity limits must be finite and non-negative");
    }
  }

  VelocityConversion convert(const geometry_msgs::Twist& input,
                             bool emergency_stop) const {
    VelocityConversion result;
    result.command = stopCommand();
    result.input_valid = isFinite(input);

    if (!result.input_valid || emergency_stop) {
      return result;
    }

    // geometry_msgs/Twist linear.x is m/s. Convert to cm/s exactly once here.
    const double x_cm_s = input.linear.x * 100.0;
    result.command.x = static_cast<float>(
        clamp(x_cm_s, -limits_.max_reverse_cm_s, limits_.max_forward_cm_s));
    result.command.y = 0.0F;
    result.command.yaw_rate = static_cast<float>(
        clamp(input.angular.z, -limits_.max_yaw_rate_rad_s,
              limits_.max_yaw_rate_rad_s));
    return result;
  }

  static puppy_control::Velocity stopCommand() {
    puppy_control::Velocity command;
    command.x = 0.0F;
    command.y = 0.0F;
    command.yaw_rate = 0.0F;
    return command;
  }

 private:
  static bool isFinite(const geometry_msgs::Twist& input) {
    return std::isfinite(input.linear.x) &&
           std::isfinite(input.linear.y) &&
           std::isfinite(input.linear.z) &&
           std::isfinite(input.angular.x) &&
           std::isfinite(input.angular.y) &&
           std::isfinite(input.angular.z);
  }

  static double clamp(double value, double minimum, double maximum) {
    return std::max(minimum, std::min(value, maximum));
  }

  VelocityLimits limits_;
};

}  // namespace ros1_maze_escape

