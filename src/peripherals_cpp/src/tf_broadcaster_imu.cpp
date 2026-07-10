// tf_broadcaster_imu.cpp
// ─────────────────────────────────────────────────────────────────────────────
// tf_broadcaster_imu.py (73줄)의 C++ 이식판 — IMU orientation 을 TF 로 방송.
//
// 원본은 쿼터니언 → 회전행렬 → 역행렬 → 쿼터니언(numpy 고유값 분해)으로
// 계산하지만, 회전행렬의 역행렬을 다시 쿼터니언으로 바꾸는 것은 수학적으로
// **원래 쿼터니언의 켤레(conjugate)** 와 같다 (단위 쿼터니언 기준).
// rot2qua 의 부호 규약(w >= 0)도 그대로 유지한다.
// ─────────────────────────────────────────────────────────────────────────────

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"

class ImuTransformBroadcaster : public rclcpp::Node
{
public:
  ImuTransformBroadcaster()
  : Node("tf_broadcaster_imu")
  {
    target_frame_ = declare_parameter<std::string>("imu_link", "imu_link");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_frame");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "imu");

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 1,
      std::bind(&ImuTransformBroadcaster::handle_imu, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "\033[1;32mstart\033[0m");
  }

private:
  void handle_imu(sensor_msgs::msg::Imu::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = get_clock()->now();
    t.header.frame_id = target_frame_;
    t.child_frame_id = imu_frame_;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    // 역회전 = 켤레 쿼터니언, w < 0 이면 전체 부호 반전 (원본 rot2qua 규약)
    double x = -msg->orientation.x;
    double y = -msg->orientation.y;
    double z = -msg->orientation.z;
    double w = msg->orientation.w;
    if (w < 0) {
      x = -x;
      y = -y;
      z = -z;
      w = -w;
    }
    t.transform.rotation.x = x;
    t.transform.rotation.y = y;
    t.transform.rotation.z = z;
    t.transform.rotation.w = w;

    broadcaster_->sendTransform(t);
  }

  std::string target_frame_;
  std::string imu_frame_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuTransformBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
