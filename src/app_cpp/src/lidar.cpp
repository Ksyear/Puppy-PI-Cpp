// lidar.cpp
// ─────────────────────────────────────────────────────────────────────────────
// app/lidar.py (175줄)의 C++ 이식판 — LiDAR 응용 (모드 1: 장애물 회피,
// 모드 2: 추적, 모드 3: 경비). /scan 을 구독해 /cmd_vel_nav 로 Twist 발행.
// 임계값/속도/타임스탬프 쿨다운 로직 전부 원본과 동일.
// ─────────────────────────────────────────────────────────────────────────────

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "puppy_control_msgs/srv/set_float64_list.hpp"
#include "puppy_control_msgs/srv/set_int64.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;
using SetInt64 = puppy_control_msgs::srv::SetInt64;
using SetFloat64List = puppy_control_msgs::srv::SetFloat64List;

namespace
{
inline double sign(double v) {return (v > 0.0) - (v < 0.0);}
inline double now_sec() {return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();}
}  // namespace

class LidarController : public rclcpp::Node
{
public:
  LidarController()
  : Node("lidar_app")
  {
    velocity_pub_ = create_publisher<Twist>("/cmd_vel_nav", 10);
    velocity_pub_->publish(Twist());

    enter_srv_ = create_service<std_srvs::srv::Trigger>(
      "/lidar_app/enter",
      std::bind(&LidarController::enter_cb, this, std::placeholders::_1, std::placeholders::_2));
    exit_srv_ = create_service<std_srvs::srv::Trigger>(
      "/lidar_app/exit",
      std::bind(&LidarController::exit_cb, this, std::placeholders::_1, std::placeholders::_2));
    heartbeat_srv_ = create_service<std_srvs::srv::SetBool>(
      "/lidar_app/heartbeat",
      std::bind(&LidarController::heartbeat_cb, this,
        std::placeholders::_1, std::placeholders::_2));
    set_running_srv_ = create_service<SetInt64>(
      "/lidar_app/set_running",
      std::bind(&LidarController::set_running_cb, this,
        std::placeholders::_1, std::placeholders::_2));
    set_parameters_srv_ = create_service<SetFloat64List>(
      "/lidar_app/adjust_parameters",
      std::bind(&LidarController::set_parameters_cb, this,
        std::placeholders::_1, std::placeholders::_2));
  }

private:
  void reset_value()
  {
    running_mode_ = 0;
    threshold_ = 0.3;   // 원본과 동일: 초기값은 0.9 지만 reset 은 0.3
    speed_ = 0.12;
    scan_angle_ = M_PI / 2;
    lidar_sub_.reset();
  }

  void enter_cb(
    std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    RCLCPP_INFO(get_logger(), "Lidar entering operation mode");
    std::lock_guard<std::mutex> lock(mutex_);
    reset_value();
    lidar_sub_ = create_subscription<LaserScan>(
      "/scan", 10,
      std::bind(&LidarController::lidar_callback, this, std::placeholders::_1));
    response->success = true;
    response->message = "Entered";
  }

  void do_exit()
  {
    RCLCPP_INFO(get_logger(), "Lidar exiting operation mode");
    std::lock_guard<std::mutex> lock(mutex_);
    reset_value();
    if (heartbeat_timer_) {
      heartbeat_timer_->cancel();
    }
  }

  void exit_cb(
    std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    do_exit();
    response->success = true;
    response->message = "Exited";
  }

  // 원본: 5초 안에 heartbeat 가 다시 오지 않으면 자동 exit
  void heartbeat_cb(
    std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    if (heartbeat_timer_) {
      heartbeat_timer_->cancel();
    }
    if (request->data) {
      heartbeat_timer_ = create_wall_timer(
        5s, [this]() {
          heartbeat_timer_->cancel();
          do_exit();
        });
    }
    response->success = request->data;
  }

  void lidar_callback(LaserScan::SharedPtr lidar_data)
  {
    // 5cm 미만은 무한대 취급 (원본과 동일)
    std::vector<float> ranges(lidar_data->ranges.begin(), lidar_data->ranges.end());
    for (auto & r : ranges) {
      if (r < 0.05f) {
        r = 9999.0f;
      }
    }
    if (ranges.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // 최솟값과 그 각도 (NaN 은 무시 — np.nanargmin 대응)
    size_t min_index = 0;
    float dist = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < ranges.size(); ++i) {
      if (!std::isnan(ranges[i]) && ranges[i] < dist) {
        dist = ranges[i];
        min_index = i;
      }
    }
    double angle = lidar_data->angle_min + lidar_data->angle_increment * min_index;
    if (angle >= M_PI) {
      angle -= 2 * M_PI;
    }

    Twist twist;
    const double now = now_sec();

    if (running_mode_ == 1 && timestamp_ <= now) {  // 장애물 회피
      if (std::abs(angle) < scan_angle_ / 2 && dist < threshold_) {
        twist.linear.x = speed_ / 6;
        twist.angular.z = speed_ * 3 * -sign(angle);
        timestamp_ = now + 0.8;
      } else {
        twist.linear.x = speed_;
        twist.angular.z = 0.0;
      }
      velocity_pub_->publish(twist);
    } else if (running_mode_ == 2 && timestamp_ <= now) {  // 추적
      if (std::abs(angle) < scan_angle_ / 2) {
        if (dist < threshold_ && std::abs(angle * 180.0 / M_PI) > 10) {
          twist.linear.x = 0.01;
          twist.angular.z = speed_ * 3 * sign(angle);
          timestamp_ = now + 0.4;
        } else if (dist < threshold_ && dist > 0.35) {
          twist.linear.x = speed_;
          twist.angular.z = 0.0;
          timestamp_ = now + 0.4;
        }
      }
      velocity_pub_->publish(twist);
    } else if (running_mode_ == 3 && timestamp_ <= now) {  // 경비
      if (dist < threshold_ && std::abs(angle * 180.0 / M_PI) > 10) {
        twist.linear.x = 0.01;
        twist.angular.z = speed_ * 3 * sign(angle);
        timestamp_ = now + 0.4;
      }
      velocity_pub_->publish(twist);
    }
  }

  void set_running_cb(
    SetInt64::Request::SharedPtr request,
    SetInt64::Response::SharedPtr response)
  {
    const int64_t mode = request->data;
    RCLCPP_INFO(get_logger(), "Setting running mode to %ld", static_cast<long>(mode));
    if (mode < 0 || mode > 3) {
      response->success = false;
      response->message = "Invalid running mode " + std::to_string(mode);
    } else {
      std::lock_guard<std::mutex> lock(mutex_);
      running_mode_ = static_cast<int>(mode);
      velocity_pub_->publish(Twist());
      response->success = true;
      response->message = "Running mode set to " + std::to_string(mode);
    }
  }

  void set_parameters_cb(
    SetFloat64List::Request::SharedPtr request,
    SetFloat64List::Response::SharedPtr response)
  {
    if (request->data.size() < 3) {
      response->success = false;
      response->message = "Need [threshold, scan_angle, speed]";
      return;
    }
    const double new_threshold = request->data[0];
    const double new_scan_angle = request->data[1];
    const double new_speed = request->data[2];
    RCLCPP_INFO(
      get_logger(), "Setting new parameters: threshold=%.2f, scan_angle=%.2f, speed=%.2f",
      new_threshold, new_scan_angle, new_speed);

    if (new_threshold < 0.3 || new_threshold > 1.5) {
      response->success = false;
      response->message = "Threshold out of range (0.3 ~ 1.5)";
    } else if (new_speed <= 0) {
      response->success = false;
      response->message = "Speed must be greater than 0";
    } else {
      std::lock_guard<std::mutex> lock(mutex_);
      threshold_ = new_threshold;
      scan_angle_ = new_scan_angle * M_PI / 180.0;
      speed_ = new_speed * 0.002;  // 원본의 스케일 그대로
      response->success = true;
      response->message = "Parameters updated successfully";
    }
  }

  int running_mode_{0};
  double threshold_{0.9};
  double scan_angle_{M_PI / 2};
  double speed_{0.12};
  double timestamp_{0.0};
  std::mutex mutex_;

  rclcpp::Publisher<Twist>::SharedPtr velocity_pub_;
  rclcpp::Subscription<LaserScan>::SharedPtr lidar_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enter_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr exit_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr heartbeat_srv_;
  rclcpp::Service<SetInt64>::SharedPtr set_running_srv_;
  rclcpp::Service<SetFloat64List>::SharedPtr set_parameters_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarController>());
  rclcpp::shutdown();
  return 0;
}
