// lidar_mapping_node.cpp
// ─────────────────────────────────────────────────────────────────────────────
// LiDAR(LD19)로 2D 점유격자 지도를 만드는 자체 C++ SLAM 노드.
//
//   /scan (sensor_msgs/LaserScan)
//      → 스캔매칭으로 자세(x,y,θ) 추정 (오도메트리 불필요 — PuppyPi 에 odom 없음)
//      → 점유격자 통합
//      → /map (nav_msgs/OccupancyGrid) 발행 + map→base TF 방송
//      → ~/save_map 서비스: slam_toolbox/map_server 와 같은 pgm+yaml 형식으로 저장
//
// 프레임/토픽/해상도는 공식 slam_toolbox 설정(src/slam/config/slam.yaml)과
// 호환: map / base_footprint / scan / 0.05m / 최대 12m.
// 알고리즘 설명과 한계는 include/lidar_mapping_cpp/mapping_core.hpp 참고.
// ─────────────────────────────────────────────────────────────────────────────

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "lidar_mapping_cpp/mapping_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;
namespace lm = lidar_mapping_cpp;

class LidarMappingNode : public rclcpp::Node
{
public:
  LidarMappingNode()
  : Node("lidar_mapping")
  {
    // ── 파라미터 ──
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "scan");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    const double resolution = declare_parameter<double>("resolution", 0.05);
    const double map_size_m = declare_parameter<double>("map_size_m", 20.0);
    max_range_ = declare_parameter<double>("max_range", 11.5);   // LD19 최대 12m
    min_range_ = declare_parameter<double>("min_range", 0.10);

    // 이 이상 움직였을 때만 지도에 통합 (제자리 노이즈 누적 방지)
    min_travel_distance_ = declare_parameter<double>("min_travel_distance", 0.05);
    min_travel_heading_ = declare_parameter<double>("min_travel_heading", 0.05);

    // LiDAR 장착 위치 보정 (base 중심에서의 오프셋)
    laser_x_ = declare_parameter<double>("laser_x", 0.0);
    laser_y_ = declare_parameter<double>("laser_y", 0.0);
    laser_yaw_ = declare_parameter<double>("laser_yaw", 0.0);

    match_params_.lin_step = declare_parameter<double>("match_lin_step", 0.05);
    match_params_.ang_step = declare_parameter<double>("match_ang_step", 0.03);

    const double map_pub_period = declare_parameter<double>("map_pub_period", 1.0);
    map_save_path_ = declare_parameter<std::string>(
      "map_save_path", "/home/ubuntu/ros2_ws/src/slam/maps");
    map_name_ = declare_parameter<std::string>("map_name", "cpp_map");

    const int cells = static_cast<int>(map_size_m / resolution);
    grid_ = std::make_unique<lm::OccGrid>(cells, cells, resolution);

    // ── ROS 인터페이스 ──
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local());  // rviz 늦게 켜도 지도 수신
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarMappingNode::scan_cb, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    map_timer_ = create_wall_timer(
      std::chrono::duration<double>(map_pub_period), [this]() {publish_map();});
    save_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/save_map",
      std::bind(&LidarMappingNode::save_map, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "LiDAR 지도작성 시작: %.0f×%.0fm @ %.2fm, scan=%s → map(frame=%s)",
      map_size_m, map_size_m, resolution, scan_topic.c_str(), map_frame_.c_str());
    RCLCPP_INFO(get_logger(), "지도 저장: ros2 service call %s/save_map std_srvs/srv/Trigger",
      get_fully_qualified_name());
  }

private:
  void scan_cb(sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // LaserScan → base 기준 점 목록 (LiDAR 장착 오프셋 반영)
    lm::ScanPoints pts;
    pts.reserve(msg->ranges.size());
    const double cl = std::cos(laser_yaw_);
    const double sl = std::sin(laser_yaw_);
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < min_range_ || r > max_range_) {
        continue;
      }
      const double a = msg->angle_min + msg->angle_increment * static_cast<double>(i);
      const double lx = r * std::cos(a);
      const double ly = r * std::sin(a);
      pts.emplace_back(
        static_cast<float>(laser_x_ + cl * lx - sl * ly),
        static_cast<float>(laser_y_ + sl * lx + cl * ly));
    }
    if (pts.size() < 30) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "유효한 스캔 점이 %zu개뿐 — LiDAR 상태 확인 필요", pts.size());
      return;
    }

    if (!initialized_) {
      lm::integrate_scan(*grid_, pose_, pts);
      last_integrated_ = pose_;
      initialized_ = true;
    } else {
      pose_ = lm::match_scan(*grid_, pose_, pts, match_params_);

      // 일정 거리/각도 이상 움직였을 때만 통합
      const double moved = std::hypot(
        pose_.x - last_integrated_.x, pose_.y - last_integrated_.y);
      double turned = std::fabs(pose_.theta - last_integrated_.theta);
      while (turned > M_PI) {turned = std::fabs(turned - 2 * M_PI);}
      if (moved >= min_travel_distance_ || turned >= min_travel_heading_) {
        lm::integrate_scan(*grid_, pose_, pts);
        last_integrated_ = pose_;
      }
    }

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = msg->header.stamp;
      t.header.frame_id = map_frame_;
      t.child_frame_id = base_frame_;
      t.transform.translation.x = pose_.x;
      t.transform.translation.y = pose_.y;
      t.transform.rotation.z = std::sin(pose_.theta / 2.0);
      t.transform.rotation.w = std::cos(pose_.theta / 2.0);
      tf_broadcaster_->sendTransform(t);
    }
  }

  void publish_map()
  {
    if (!initialized_) {
      return;
    }
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = map_frame_;
    msg.info.resolution = static_cast<float>(grid_->resolution());
    msg.info.width = grid_->width();
    msg.info.height = grid_->height();
    msg.info.origin.position.x = grid_->origin_x();
    msg.info.origin.position.y = grid_->origin_y();
    msg.info.origin.orientation.w = 1.0;
    msg.data.resize(static_cast<size_t>(grid_->width()) * grid_->height());
    for (int cy = 0; cy < grid_->height(); ++cy) {
      for (int cx = 0; cx < grid_->width(); ++cx) {
        msg.data[static_cast<size_t>(cy) * grid_->width() + cx] = grid_->occupancy_at(cx, cy);
      }
    }
    map_pub_->publish(msg);
  }

  // map_server(nav2) 형식으로 저장: <경로>/<이름>.pgm + .yaml
  void save_map(
    std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    const std::string pgm_path = map_save_path_ + "/" + map_name_ + ".pgm";
    const std::string yaml_path = map_save_path_ + "/" + map_name_ + ".yaml";

    FILE * f = std::fopen(pgm_path.c_str(), "wb");
    if (!f) {
      response->success = false;
      response->message = "저장 실패 (경로 확인): " + pgm_path;
      return;
    }
    std::fprintf(f, "P5\n%d %d\n255\n", grid_->width(), grid_->height());
    // PGM 첫 행 = 지도 위쪽(y 최대) — map_server 규약: 점유=0, 빈=254, 미탐사=205
    for (int cy = grid_->height() - 1; cy >= 0; --cy) {
      for (int cx = 0; cx < grid_->width(); ++cx) {
        const int8_t occ = grid_->occupancy_at(cx, cy);
        uint8_t pixel = 205;
        if (occ >= 0 && occ <= 25) {
          pixel = 254;
        } else if (occ >= 65) {
          pixel = 0;
        }
        std::fputc(pixel, f);
      }
    }
    std::fclose(f);

    FILE * y = std::fopen(yaml_path.c_str(), "w");
    if (!y) {
      response->success = false;
      response->message = "yaml 저장 실패: " + yaml_path;
      return;
    }
    std::fprintf(
      y,
      "image: %s.pgm\nresolution: %.6f\norigin: [%.6f, %.6f, 0.000000]\n"
      "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
      map_name_.c_str(), grid_->resolution(), grid_->origin_x(), grid_->origin_y());
    std::fclose(y);

    response->success = true;
    response->message = "저장 완료: " + pgm_path;
    RCLCPP_INFO(get_logger(), "지도 저장 완료: %s", pgm_path.c_str());
  }

  // ── 상태 ──
  std::unique_ptr<lm::OccGrid> grid_;
  lm::Pose2D pose_;
  lm::Pose2D last_integrated_;
  bool initialized_{false};
  lm::MatchParams match_params_;

  // ── 파라미터 ──
  std::string map_frame_, base_frame_;
  bool publish_tf_{true};
  double max_range_{11.5}, min_range_{0.1};
  double min_travel_distance_{0.05}, min_travel_heading_{0.05};
  double laser_x_{0.0}, laser_y_{0.0}, laser_yaw_{0.0};
  std::string map_save_path_, map_name_;

  // ── ROS 인터페이스 ──
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarMappingNode>());
  rclcpp::shutdown();
  return 0;
}
