#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <boost/bind/bind.hpp>
#include <geometry_msgs/Twist.h>
#include <puppy_control/Velocity.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "ros1_maze_escape/velocity_adapter.hpp"

namespace ros1_maze_escape {

class CmdVelAdapterNode {
 public:
  CmdVelAdapterNode() : private_nh_("~") {
    std::string input_topic;
    std::string output_topic;
    std::string emergency_stop_topic;
    private_nh_.param<std::string>("input_topic", input_topic, "/cmd_vel");
    private_nh_.param<std::string>(
        "output_topic", output_topic, "/puppy_control/velocity/autogait");
    private_nh_.param<std::string>(
        "emergency_stop_topic", emergency_stop_topic, "/emergency_stop");
    private_nh_.param("max_forward_speed_cm_s", limits_.max_forward_cm_s, 10.0);
    private_nh_.param("max_reverse_speed_cm_s", limits_.max_reverse_cm_s, 10.0);
    private_nh_.param("max_yaw_rate_rad_s", limits_.max_yaw_rate_rad_s, 0.28);
    private_nh_.param("watchdog_timeout_s", watchdog_timeout_s_, 0.5);

    if (!std::isfinite(watchdog_timeout_s_) || watchdog_timeout_s_ <= 0.0) {
      throw std::invalid_argument("watchdog_timeout_s must be finite and positive");
    }

    adapter_ = std::make_unique<VelocityAdapter>(limits_);
    velocity_pub_ = nh_.advertise<puppy_control::Velocity>(output_topic, 1);
    cmd_vel_sub_ = nh_.subscribe(
        input_topic, 10, &CmdVelAdapterNode::cmdVelCallback, this);
    emergency_stop_sub_ = nh_.subscribe(
        emergency_stop_topic, 10,
        &CmdVelAdapterNode::emergencyStopCallback, this);

    const double timer_period_s = std::min(0.05, watchdog_timeout_s_ / 2.0);
    watchdog_timer_ = nh_.createTimer(
        ros::Duration(timer_period_s), &CmdVelAdapterNode::watchdogCallback, this);
    ros::on_shutdown(
        boost::bind(&CmdVelAdapterNode::publishShutdownStops, this));

    last_cmd_time_ = ros::SteadyTime::now();
    publishStop("startup");
    ROS_INFO_STREAM("cmd_vel_adapter ready: input=" << input_topic
                    << " output=" << output_topic
                    << " emergency_stop=" << emergency_stop_topic
                    << " forward_limit=" << limits_.max_forward_cm_s << " cm/s"
                    << " reverse_limit=" << limits_.max_reverse_cm_s << " cm/s"
                    << " yaw_limit=" << limits_.max_yaw_rate_rad_s << " rad/s"
                    << " watchdog=" << watchdog_timeout_s_ << " s");
  }

 private:
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& message) {
    last_cmd_time_ = ros::SteadyTime::now();
    watchdog_stopped_ = false;

    const VelocityConversion conversion =
        adapter_->convert(*message, emergency_stop_active_);
    ROS_DEBUG_STREAM("[RAW_TWIST] linear=("
                     << message->linear.x << ", " << message->linear.y << ", "
                     << message->linear.z << ") angular=("
                     << message->angular.x << ", " << message->angular.y << ", "
                     << message->angular.z << ")");

    if (!conversion.input_valid) {
      ROS_ERROR_THROTTLE(
          1.0, "[RAW_TWIST] NaN or Inf detected; publishing stop");
    } else if (emergency_stop_active_) {
      ROS_WARN_THROTTLE(
          1.0, "emergency_stop is active; velocity command is forced to zero");
    }

    publishCommand(conversion.command);
  }

  void emergencyStopCallback(const std_msgs::Bool::ConstPtr& message) {
    const bool was_active = emergency_stop_active_;
    emergency_stop_active_ = message->data;
    if (emergency_stop_active_) {
      publishStop("emergency_stop");
      if (!was_active) {
        ROS_ERROR("emergency_stop activated");
      }
      return;
    }

    if (was_active) {
      // Require a fresh /cmd_vel after release; do not replay an old command.
      watchdog_stopped_ = true;
      publishStop("emergency_stop released; waiting for fresh cmd_vel");
      ROS_WARN("emergency_stop released; a fresh cmd_vel is required");
    }
  }

  void watchdogCallback(const ros::TimerEvent&) {
    if (emergency_stop_active_) {
      return;
    }

    const bool timed_out =
        (ros::SteadyTime::now() - last_cmd_time_).toSec() >=
        watchdog_timeout_s_;
    if (timed_out && !watchdog_stopped_) {
      watchdog_stopped_ = true;
      publishStop("cmd_vel watchdog timeout");
      ROS_WARN_STREAM("No cmd_vel received for " << watchdog_timeout_s_
                      << " s; stop command published");
    }
  }

  void publishCommand(const puppy_control::Velocity& command) {
    velocity_pub_.publish(command);
    ROS_DEBUG_STREAM("[PUPPY_COMMAND] x=" << command.x << " cm/s"
                     << " y=" << command.y << " cm/s"
                     << " yaw_rate=" << command.yaw_rate << " rad/s");
  }

  void publishStop(const std::string& reason) {
    publishCommand(VelocityAdapter::stopCommand());
    ROS_DEBUG_STREAM("[PUPPY_COMMAND] stop reason=" << reason);
  }

  void publishShutdownStops() {
    if (shutdown_stops_published_) {
      return;
    }
    shutdown_stops_published_ = true;
    const puppy_control::Velocity stop = VelocityAdapter::stopCommand();
    for (int count = 0; count < 3; ++count) {
      velocity_pub_.publish(stop);
      ros::WallDuration(0.02).sleep();
    }
    ROS_INFO("[PUPPY_COMMAND] published 3 shutdown stop commands");
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher velocity_pub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber emergency_stop_sub_;
  ros::Timer watchdog_timer_;

  VelocityLimits limits_;
  std::unique_ptr<VelocityAdapter> adapter_;
  double watchdog_timeout_s_{0.5};
  ros::SteadyTime last_cmd_time_;
  bool watchdog_stopped_{false};
  bool emergency_stop_active_{false};
  bool shutdown_stops_published_{false};
};

}  // namespace ros1_maze_escape

int main(int argc, char** argv) {
  ros::init(argc, argv, "cmd_vel_adapter");
  try {
    ros1_maze_escape::CmdVelAdapterNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("cmd_vel_adapter initialization failed: " << error.what());
    return 1;
  }
  return 0;
}
