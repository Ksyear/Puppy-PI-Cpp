// puppy_control_node.cpp
// ─────────────────────────────────────────────────────────────────────────────
// puppy_control(puppy.py, 851줄)의 C++ 이식판 — ROS 래퍼 완전 이식.
//
// 토픽/서비스/파라미터/검증 규칙/IMU 상보필터/100Hz 발행 루프를 파이썬 원본과
// 동일하게 유지한다 (드롭인 교체 목적). 보행 엔진 호출은 PuppyEngine 인터페이스
// 뒤로 분리 — 현재는 StubPuppyEngine(서보 출력 없음)이며, 로봇의
// 실제 엔진은 사용자가 보유한 system image를 기반으로 로컬 private/에만 두며
// 공개 저장소에는 포함하지 않는다.
//
// 파이썬 원본과 의도적으로 다른 점:
//  - 로봇팔(ArmIK) 초기화 생략: 원본은 with_arm=0 이어도 ArmIK 를 만들고
//    setPitchRangeMoving 을 호출하지만, 팔 IK 는 파이썬 sdk 에 있고 팔 없는
//    구성에서는 부작용이 서보 9번 이동뿐이라 set_servo_pulse(9,1500,300) 만 유지
//  - pub() 안의 2ms 보정 sleep 생략 (rclpy 타이밍 보정용 핵)
//  - GaitPCFun 에 data 길이 검사 추가 (원본은 짧은 배열이 오면 예외)
// ─────────────────────────────────────────────────────────────────────────────

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "puppy_control_cpp/puppy_engine.hpp"
#include "puppy_control_msgs/msg/gait.hpp"
#include "puppy_control_msgs/msg/pose.hpp"
#include "puppy_control_msgs/msg/set_servo.hpp"
#include "puppy_control_msgs/msg/velocity.hpp"
#include "puppy_control_msgs/srv/set_run_action_name.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_robot_controller_msgs/msg/buzzer_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
namespace pc = puppy_control_cpp;

using GaitMsg = puppy_control_msgs::msg::Gait;
using PoseMsg = puppy_control_msgs::msg::Pose;
using SetServoMsg = puppy_control_msgs::msg::SetServo;
using VelocityMsg = puppy_control_msgs::msg::Velocity;
using SetRunActionName = puppy_control_msgs::srv::SetRunActionName;
using BuzzerState = ros_robot_controller_msgs::msg::BuzzerState;

namespace
{

constexpr char ROS_NODE_NAME[] = "puppy_control";

inline double radians(double deg) {return deg * M_PI / 180.0;}
inline double sign(double v) {return (v > 0.0) - (v < 0.0);}

// 자세 프리셋 (원본의 PuppyPose 딕셔너리와 동일 필드)
struct PosePreset
{
  double roll{0.0}, pitch{0.0}, yaw{0.0};
  double height{-10.0}, x_shift{-0.5};
  double stance_x{0.0}, stance_y{0.0};
};

// 보행 설정 (원본의 GaitConfig 딕셔너리와 동일 필드)
struct GaitPreset
{
  double overlap_time{0.1}, swing_time{0.15}, clearance_time{0.0}, z_clearance{5.0};
};

// 원본 MPU6050._SecondOrderFilter() 클로저의 이식판 (2차 상보필터)
class SecondOrderFilter
{
public:
  double step(double angle_m, double gyro_m, double dt = 0.01)
  {
    const double k = 1.0 - K2_;
    const double x1 = (angle_m - angle_) * k * k;
    y1_ += x1 * dt;
    const double x2 = y1_ + 2.0 * k * (angle_m - angle_) + gyro_m;
    angle_ += x2 * dt;
    return angle_;
  }

private:
  static constexpr double K2_ = 0.02;
  double y1_{0.0};
  double angle_{0.0};
};

}  // namespace

// 원본 MPU6050 클래스: /ros_robot_controller/imu_raw 구독 + 상보필터로 roll/pitch 계산
class Mpu6050
{
public:
  explicit Mpu6050(rclcpp::Node * node)
  {
    sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
      "/ros_robot_controller/imu_raw", 10,
      [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        accel_[0] = msg->linear_acceleration.x;
        accel_[1] = msg->linear_acceleration.y;
        accel_[2] = msg->linear_acceleration.z;
        gyro_[0] = msg->angular_velocity.x;
        gyro_[1] = msg->angular_velocity.y;
        gyro_[2] = msg->angular_velocity.z;
      });
  }

  pc::EulerAngle get_euler_angle(double dt = 0.01)
  {
    const double accel_y = std::atan2(accel_[0], accel_[2]) * 180.0 / M_PI;
    const double angle_y = filter_y_.step(-accel_y, gyro_[1], dt);

    const double accel_x = std::atan2(accel_[1], accel_[2]) * 180.0 / M_PI;
    const double angle_x = filter_x_.step(accel_x, gyro_[0], dt);

    pc::EulerAngle e;
    e.pitch = -radians(angle_x);
    e.roll = -radians(angle_y);
    e.yaw = 0.0;
    return e;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  std::array<double, 3> accel_{{0, 0, 0}};
  std::array<double, 3> gyro_{{0, 0, 0}};
  SecondOrderFilter filter_x_;
  SecondOrderFilter filter_y_;
};

namespace puppy_control_cpp
{
std::unique_ptr<PuppyEngine> make_puppy_engine();  // stub_engine.cpp 에서 제공
}

class PuppyControlNode : public rclcpp::Node
{
public:
  PuppyControlNode()
  : Node(ROS_NODE_NAME)
  {
    declare_all_parameters();
    load_presets_from_parameters();

    engine_ = pc::make_puppy_engine();
    mpu_ = std::make_unique<Mpu6050>(this);

    // 현재 자세/보행의 초기값: 원본과 동일하게 자세는 Stand, 보행은 current(=Fast)
    cur_pose_ = pose_presets_["Stand"];
    cur_gait_ = gait_presets_["current"];

    // 초기 자세는 LieDown 으로 구성 (원본 __init__ 과 동일)
    const auto & lie = pose_presets_["LieDown"];
    engine_->stance_config(
      stance(lie.stance_x, lie.stance_y, lie.height, lie.x_shift), lie.pitch, lie.roll);
    engine_->gait_config(
      cur_gait_.overlap_time, cur_gait_.swing_time,
      cur_gait_.clearance_time, cur_gait_.z_clearance);

    engine_->start();
    engine_->move_stop(500);
    std::this_thread::sleep_for(500ms);
    engine_->set_servo_pulse(9, 1500, 300);  // 원본: setServoPulse(9, 1500, 300)
    std::this_thread::sleep_for(300ms);

    // ── 서비스 ──
    srv_set_running_ = create_service<std_srvs::srv::SetBool>(
      std::string("/") + ROS_NODE_NAME + "/set_running",
      std::bind(&PuppyControlNode::set_running, this,
        std::placeholders::_1, std::placeholders::_2));
    srv_go_home_ = create_service<std_srvs::srv::Empty>(
      std::string("/") + ROS_NODE_NAME + "/go_home",
      std::bind(&PuppyControlNode::go_home_srv, this,
        std::placeholders::_1, std::placeholders::_2));
    srv_self_balancing_ = create_service<std_srvs::srv::SetBool>(
      std::string("/") + ROS_NODE_NAME + "/set_self_balancing",
      std::bind(&PuppyControlNode::set_self_balancing, this,
        std::placeholders::_1, std::placeholders::_2));
    srv_run_action_ = create_service<SetRunActionName>(
      std::string("/") + ROS_NODE_NAME + "/runActionGroup",
      std::bind(&PuppyControlNode::run_action_group, this,
        std::placeholders::_1, std::placeholders::_2));
    srv_mark_time_ = create_service<std_srvs::srv::SetBool>(
      std::string("/") + ROS_NODE_NAME + "/set_mark_time",
      std::bind(&PuppyControlNode::set_mark_time, this,
        std::placeholders::_1, std::placeholders::_2));

    // ── 구독 ──
    sub_gait_ = create_subscription<GaitMsg>(
      std::string("/") + ROS_NODE_NAME + "/gait", 10,
      std::bind(&PuppyControlNode::gait_cb, this, std::placeholders::_1));
    sub_velocity_ = create_subscription<VelocityMsg>(
      std::string("/") + ROS_NODE_NAME + "/velocity", 10,
      std::bind(&PuppyControlNode::velocity_cb, this, std::placeholders::_1));
    sub_velocity_move_ = create_subscription<VelocityMsg>(
      std::string("/") + ROS_NODE_NAME + "/velocity_move", 10,
      std::bind(&PuppyControlNode::velocity_move_cb, this, std::placeholders::_1));
    sub_velocity_autogait_ = create_subscription<VelocityMsg>(
      std::string("/") + ROS_NODE_NAME + "/velocity/autogait", 10,
      std::bind(&PuppyControlNode::velocity_autogait_cb, this, std::placeholders::_1));
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&PuppyControlNode::cmd_vel_cb, this, std::placeholders::_1));
    sub_cmd_vel_nav_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_nav", 10,
      std::bind(&PuppyControlNode::cmd_vel_nav_cb, this, std::placeholders::_1));
    sub_pose_ = create_subscription<PoseMsg>(
      std::string("/") + ROS_NODE_NAME + "/pose", 10,
      std::bind(&PuppyControlNode::pose_cb, this, std::placeholders::_1));
    sub_four_legs_ = create_subscription<geometry_msgs::msg::Polygon>(
      std::string("/") + ROS_NODE_NAME + "/fourLegsRelativeCoordControl", 10,
      std::bind(&PuppyControlNode::four_legs_cb, this, std::placeholders::_1));
    sub_gait_pc_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      std::string("/") + ROS_NODE_NAME + "/gait/pc", 10,
      std::bind(&PuppyControlNode::gait_pc_cb, this, std::placeholders::_1));
    sub_set_servo_ = create_subscription<SetServoMsg>(
      std::string("/") + ROS_NODE_NAME + "/setServo", 10,
      std::bind(&PuppyControlNode::set_servo_cb, this, std::placeholders::_1));

    // ── 발행 ──
    legs_coord_pub_ = create_publisher<geometry_msgs::msg::Polygon>(
      std::string("/") + ROS_NODE_NAME + "/legs_coord", 10);
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    joint_state_.name = {
      "rf_joint1", "lf_joint1", "rb_joint1", "lb_joint1",
      "rf_joint2", "lf_joint2", "rb_joint2", "lb_joint2"};
    for (int i = 1; i <= 8; ++i) {
      joint_controller_pubs_.push_back(
        create_publisher<std_msgs::msg::Float64>(
          "/puppy/joint" + std::to_string(i) + "_position_controller/command", 10));
    }

    buzzer_pub_ = create_publisher<BuzzerState>("/ros_robot_controller/set_buzzer", 10);
    std::this_thread::sleep_for(200ms);
    BuzzerState buzzer;
    buzzer.freq = 1900;
    buzzer.on_time = 0.1f;
    buzzer.off_time = 0.9f;
    buzzer.repeat = 1;
    buzzer_pub_->publish(buzzer);

    // 100Hz 발행 루프 (원본 create_timer(0.01, self.pub))
    timer_ = create_wall_timer(10ms, std::bind(&PuppyControlNode::pub_tick, this));

    joint_state_pub_topic_ =
      get_parameter("joint_state_pub_topic").as_string();
    joint_state_controller_pub_topic_ =
      get_parameter("joint_state_controller_pub_topic").as_string();
  }

private:
  // ── 파라미터 선언/로드 (원본의 60여 개 declare_parameter 와 동일 이름) ──
  void declare_pose_params(const std::string & name, const PosePreset & def)
  {
    const std::string p = "PuppyPose_" + name + "_";
    declare_parameter<double>(p + "roll", def.roll);
    declare_parameter<double>(p + "pitch", def.pitch);
    declare_parameter<double>(p + "yaw", def.yaw);
    declare_parameter<double>(p + "height", def.height);
    declare_parameter<double>(p + "x_shift", def.x_shift);
    // 원본은 stance_x/stance_y 를 int 로 선언·판독 (integer_value)
    declare_parameter<int>(p + "stance_x", static_cast<int>(def.stance_x));
    declare_parameter<int>(p + "stance_y", static_cast<int>(def.stance_y));
  }

  PosePreset load_pose_params(const std::string & name)
  {
    const std::string p = "PuppyPose_" + name + "_";
    PosePreset out;
    out.roll = get_parameter(p + "roll").as_double();
    out.pitch = get_parameter(p + "pitch").as_double();
    out.yaw = get_parameter(p + "yaw").as_double();
    out.height = get_parameter(p + "height").as_double();
    out.x_shift = get_parameter(p + "x_shift").as_double();
    out.stance_x = static_cast<double>(get_parameter(p + "stance_x").as_int());
    out.stance_y = static_cast<double>(get_parameter(p + "stance_y").as_int());
    return out;
  }

  void declare_gait_params(const std::string & prefix, const GaitPreset & def)
  {
    declare_parameter<double>(prefix + "_overlap_time", def.overlap_time);
    declare_parameter<double>(prefix + "_swing_time", def.swing_time);
    declare_parameter<double>(prefix + "_clearance_time", def.clearance_time);
    declare_parameter<double>(prefix + "_z_clearance", def.z_clearance);
  }

  GaitPreset load_gait_params(const std::string & prefix)
  {
    GaitPreset out;
    out.overlap_time = get_parameter(prefix + "_overlap_time").as_double();
    out.swing_time = get_parameter(prefix + "_swing_time").as_double();
    out.clearance_time = get_parameter(prefix + "_clearance_time").as_double();
    out.z_clearance = get_parameter(prefix + "_z_clearance").as_double();
    return out;
  }

  void declare_all_parameters()
  {
    // 기본값은 puppy.py 모듈 상단의 상수와 동일 (with_arm=0 기준)
    declare_pose_params("Stand", {0.0, 0.0, 0.0, -10.0, -0.5, 0, 0});
    declare_pose_params("LieDown", {0.0, 0.0, 0.0, -5.0, 2.0, 0, 0});
    declare_pose_params("LookDown", {0.0, radians(-15.0), 0.0, -10.0, -0.5, 0, 0});
    declare_pose_params("LookDown_10deg", {0.0, radians(-10.0), 0.0, -9.0, -0.1, 0, 0});
    declare_pose_params("LookDown_20deg", {0.0, radians(-20.0), 0.0, -9.0, -0.1, 0, 0});
    declare_pose_params("LookDown_30deg", {0.0, radians(-30.0), 0.0, -9.6, -1.4, 1, 0});
    declare_pose_params("StandLow", {0.0, 0.0, 0.0, -7.0, -0.5, 0, 0});

    declare_gait_params("GaitConfigFast", {0.1, 0.15, 0.0, 5.0});
    declare_gait_params("GaitConfigSlow", {0.4, 0.3, 0.26, 4.0});
    declare_gait_params("GaitConfigMarkTime", {0.2, 0.1, 0.0, 5.0});
    declare_gait_params("GaitConfig_current", {0.1, 0.15, 0.0, 5.0});

    declare_parameter<std::string>("joint_state_pub_topic", "false");
    declare_parameter<std::string>("joint_state_controller_pub_topic", "false");
  }

  void load_presets_from_parameters()
  {
    for (const auto & name :
      {"Stand", "LieDown", "LookDown", "LookDown_10deg",
        "LookDown_20deg", "LookDown_30deg", "StandLow"})
    {
      pose_presets_[name] = load_pose_params(name);
    }
    gait_presets_["Fast"] = load_gait_params("GaitConfigFast");
    gait_presets_["Slow"] = load_gait_params("GaitConfigSlow");
    gait_presets_["MarkTime"] = load_gait_params("GaitConfigMarkTime");
    gait_presets_["current"] = load_gait_params("GaitConfig_current");
  }

  // 원본 stance(): 열 순서 [x+xs, x+xs, -x+xs, -x+xs] / [y…] / [z…]
  pc::Mat34 stance(double x, double y, double z, double x_shift) const
  {
    pc::Mat34 m = pc::zero_mat34();
    m[0] = {x + x_shift, x + x_shift, -x + x_shift, -x + x_shift};
    m[1] = {y, y, y, y};
    m[2] = {z, z, z, z};
    return m;
  }

  void apply_cur_pose_stance(double x_shift_delta = 0.0)
  {
    engine_->stance_config(
      stance(cur_pose_.stance_x, cur_pose_.stance_y, cur_pose_.height,
        cur_pose_.x_shift + x_shift_delta),
      cur_pose_.pitch, cur_pose_.roll);
  }

  // ── 토픽 핸들러 (원본과 동일 로직) ──

  void set_servo_cb(SetServoMsg::SharedPtr msg)
  {
    const int pulse = std::max(500, std::min(static_cast<int>(msg->pulse), 2500));
    const int time_ms = std::max(0, std::min(static_cast<int>(msg->time), 30000));
    engine_->set_servo_pulse(msg->id, pulse, time_ms);
  }

  void gait_cb(GaitMsg::SharedPtr msg)
  {
    cur_gait_ = {msg->overlap_time, msg->swing_time, msg->clearance_time, msg->z_clearance};
    engine_->gait_config(
      cur_gait_.overlap_time, cur_gait_.swing_time,
      cur_gait_.clearance_time, cur_gait_.z_clearance);
  }

  void gait_pc_cb(std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 6) {
      RCLCPP_WARN(get_logger(), "gait/pc: data 길이 %zu < 6 → 무시", msg->data.size());
      return;
    }
    const float gait_type = msg->data[0];
    if (gait_type == 0) {
      engine_->move_stop(100);
      apply_cur_pose_stance();
      return;
    }
    const double cycle = msg->data[2];
    if (gait_type == 1) {  // Trot
      cur_gait_.overlap_time = cycle / 4;
      cur_gait_.swing_time = cycle / 4;
      cur_gait_.clearance_time = 0;
    } else if (gait_type == 2) {  // Amble
      cur_gait_.overlap_time = cycle / 5;
      cur_gait_.swing_time = cycle / 5;
      cur_gait_.clearance_time = cycle / 10;
    } else if (gait_type == 3) {  // Walk
      cur_gait_.overlap_time = cycle / 6;
      cur_gait_.swing_time = cycle / 6;
      cur_gait_.clearance_time = cycle / 6;
    }
    cur_gait_.z_clearance = msg->data[1];
    engine_->gait_config(
      cur_gait_.overlap_time, cur_gait_.swing_time,
      cur_gait_.clearance_time, cur_gait_.z_clearance);
    apply_velocity(msg->data[3], msg->data[4], msg->data[5]);
  }

  void cmd_vel_cb(geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (std::abs(msg->linear.x) > 0.5 || std::abs(msg->angular.z) > 0.5) {
      cur_pose_ = pose_presets_["Stand"];
      apply_cur_pose_stance();
      if (std::abs(msg->linear.x) > std::abs(msg->angular.z)) {
        apply_velocity(16.0 * sign(msg->linear.x), 0.0, 0.0);
      } else {
        apply_velocity(0.0, 0.0, radians(25.0) * sign(msg->angular.z));
      }
    } else if (msg->linear.x == 0 && msg->angular.z == 0) {
      apply_velocity(0.0, 0.0, 0.0);
    }
  }

  void cmd_vel_nav_cb(geometry_msgs::msg::Twist::SharedPtr msg)
  {
    cur_pose_ = pose_presets_["Stand"];
    apply_cur_pose_stance();
    apply_velocity(msg->linear.x * 100.0, 0.0, msg->angular.z);
  }

  void velocity_move_cb(VelocityMsg::SharedPtr msg)
  {
    if (msg->x == 0 && msg->y == 0 && msg->yaw_rate == 0) {
      engine_->move_stop(100);
      apply_cur_pose_stance();
    } else {
      engine_->move(msg->x, msg->y, msg->yaw_rate);
    }
  }

  void velocity_cb(VelocityMsg::SharedPtr msg)
  {
    apply_velocity(msg->x, msg->y, msg->yaw_rate);
  }

  // 원본 VelocityFun: -999 = 자세만 초기화, 0 = 정지, 범위 초과 = 조용히 무시
  void apply_velocity(double x, double y, double yaw_rate)
  {
    if (x == -999) {
      engine_->move(0.0, 0.0, 0.0);
      apply_cur_pose_stance();
    } else if (x == 0 && y == 0 && yaw_rate == 0) {
      engine_->move_stop(100);
      apply_cur_pose_stance();
    } else if (std::abs(x) <= 35 && std::abs(y) == 0 && std::abs(yaw_rate) <= radians(51)) {
      // 전진 시 무게중심을 앞으로, 후진 시 뒤로 0.8cm 이동
      apply_cur_pose_stance(x > 0 ? -0.8 : 0.8);
      engine_->move(x, y, yaw_rate);
    }
  }

  // 원본 VelocityAutogaitFun: 속도 크기에 맞춰 보행 타이밍 자동 계산
  void velocity_autogait_cb(VelocityMsg::SharedPtr msg)
  {
    const double x = msg->x;
    const double y = msg->y;
    const double yaw = msg->yaw_rate;

    if (x == 0 && y == 0 && yaw == 0) {
      engine_->move_stop(100);
      apply_cur_pose_stance();
      return;
    }
    if (!(std::abs(x) <= 35 && std::abs(y) == 0 && std::abs(yaw) <= radians(51))) {
      return;  // 범위 초과 → 원본과 동일하게 조용히 무시
    }

    double overlap_x, swing_x, clearance_x;
    if (std::abs(x) <= 10) {
      overlap_x = 0.45 - std::abs(x) * 0.023;
      swing_x = 0.38 - std::abs(x) * 0.0154;
      clearance_x = swing_x - 0.04;
    } else if (std::abs(x) <= 15) {
      overlap_x = 0.45 - std::abs(x) * 0.023;
      swing_x = 0.38 - std::abs(x) * 0.0154;
      clearance_x = 0;
    } else {
      overlap_x = 0.1;
      swing_x = 0.15;
      clearance_x = 0;
    }

    double overlap_yaw, swing_yaw, clearance_yaw;
    if (std::abs(yaw) <= radians(10)) {
      overlap_yaw = 0.23 - std::abs(yaw) * 0.37;
      swing_yaw = 0.36 - std::abs(yaw) * 0.74;
      clearance_yaw = swing_yaw - 0.04;
    } else if (std::abs(yaw) <= radians(20)) {
      overlap_yaw = 0.23 - std::abs(yaw) * 0.37;
      swing_yaw = 0.41 - std::abs(yaw) * 0.74;
      clearance_yaw = 0;
    } else {
      overlap_yaw = 0.1;
      swing_yaw = 0.15;
      clearance_yaw = 0;
    }

    cur_gait_.overlap_time = std::min(overlap_x, overlap_yaw);
    cur_gait_.swing_time = std::min(swing_x, swing_yaw);
    cur_gait_.clearance_time = std::min(clearance_x, clearance_yaw);

    engine_->gait_config(
      cur_gait_.overlap_time, cur_gait_.swing_time,
      cur_gait_.clearance_time, cur_gait_.z_clearance);

    apply_cur_pose_stance(x > 0 ? -0.8 : 0.8);
    engine_->move(x, y, yaw);
  }

  void pose_cb(PoseMsg::SharedPtr msg)
  {
    if (std::abs(msg->roll) <= radians(31) && std::abs(msg->pitch) <= radians(31) &&
      std::abs(msg->yaw) == 0 && -15 <= msg->height && msg->height <= -5 &&
      std::abs(msg->stance_x) <= 5 && std::abs(msg->stance_y) <= 5 &&
      std::abs(msg->x_shift) <= 10)
    {
      if (msg->run_time != 0) {
        engine_->move_stop(msg->run_time);
        std::this_thread::sleep_for(10ms);
        engine_->servo_force_run();
      }
      cur_pose_ = {msg->roll, msg->pitch, msg->yaw, msg->height,
        msg->x_shift, msg->stance_x, msg->stance_y};
      apply_cur_pose_stance();
    }
  }

  void four_legs_cb(geometry_msgs::msg::Polygon::SharedPtr msg)
  {
    pc::Mat34 foot = pc::zero_mat34();
    const size_t n = std::min<size_t>(msg->points.size(), 4);
    for (size_t i = 0; i < n; ++i) {
      foot[0][i] = msg->points[i].x;
      foot[1][i] = msg->points[i].y;
      foot[2][i] = msg->points[i].z;
    }
    const pc::Mat34 joint_angles = engine_->four_legs_relative_coord_control(foot);
    engine_->send_servo_angle(joint_angles);
  }

  // ── 서비스 핸들러 ──

  void run_action_group(
    SetRunActionName::Request::SharedPtr request,
    SetRunActionName::Response::SharedPtr response)
  {
    engine_->run_action_group(request->name, request->wait);
    response->success = true;
    response->message = request->name;
  }

  void set_running(
    std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    engine_->move_stop(500);
    if (request->data) {
      engine_->start();
    } else {
      engine_->end();
    }
    response->success = true;
    response->message = "set_running";
  }

  void set_self_balancing(
    std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    if (request->data) {
      cur_pose_ = pose_presets_["StandLow"];
      apply_cur_pose_stance();
      engine_->move_stop(500);
      std::this_thread::sleep_for(10ms);
      engine_->servo_force_run();
      std::this_thread::sleep_for(500ms);
      engine_->move_stop(0);
      engine_->set_imu([this](double dt) {return mpu_->get_euler_angle(dt);});
    } else {
      engine_->set_imu(nullptr);
    }
    response->success = true;
    response->message = "set_self_balancing";
  }

  void set_mark_time(
    std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    if (request->data) {
      go_home();
      cur_gait_ = load_gait_params("GaitConfigMarkTime");
      engine_->gait_config(
        cur_gait_.overlap_time, cur_gait_.swing_time,
        cur_gait_.clearance_time, cur_gait_.z_clearance);
      engine_->move(0.0, 0.0, 0.0);
    }
    response->success = true;
    response->message = "set_mark_time";
  }

  void go_home_srv(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    go_home();
  }

  void go_home()
  {
    cur_pose_ = pose_presets_["Stand"];
    apply_cur_pose_stance();
    engine_->move_stop(500);
    std::this_thread::sleep_for(10ms);
    engine_->servo_force_run();
    std::this_thread::sleep_for(500ms);
    engine_->move_stop(0);
  }

  // ── 100Hz 발행 루프 (원본 pub) ──
  void pub_tick()
  {
    const pc::Mat34 coord = engine_->get_coord();

    if (times_ >= 100) {  // 약 1초마다 발끝 좌표 발행 + 파라미터 갱신
      times_ = 0;
      geometry_msgs::msg::Polygon poly;
      for (int i = 0; i < 4; ++i) {
        geometry_msgs::msg::Point32 p;
        p.x = static_cast<float>(coord[0][i]);
        p.y = static_cast<float>(coord[1][i]);
        p.z = static_cast<float>(coord[2][i]);
        poly.points.push_back(p);
      }
      legs_coord_pub_->publish(poly);

      joint_state_pub_topic_ = get_parameter("joint_state_pub_topic").as_string();
      joint_state_controller_pub_topic_ =
        get_parameter("joint_state_controller_pub_topic").as_string();
    }
    ++times_;

    const bool pub_joint = str_true(joint_state_pub_topic_);
    const bool pub_controller = str_true(joint_state_controller_pub_topic_);
    if (!pub_joint && !pub_controller) {
      return;
    }

    // 좌표(cm)를 m 로 바꿔 관절각 계산 (원본: coord / 100)
    pc::Mat34 coord_m = coord;
    for (auto & row : coord_m) {
      for (auto & v : row) {
        v /= 100.0;
      }
    }
    const pc::Mat34 joint_angles = engine_->four_legs_relative_coord_control(coord_m);

    // data = joint_angles 1행(4개) + 2행(4개)
    std::vector<double> data;
    data.reserve(8);
    for (int i = 0; i < 4; ++i) {data.push_back(joint_angles[1][i]);}
    for (int i = 0; i < 4; ++i) {data.push_back(joint_angles[2][i]);}

    joint_state_.header.stamp = get_clock()->now();
    for (size_t i = 0; i < data.size(); ++i) {
      if (i > 3) {
        // 원본의 무릎 관절 보정 다항식 (링크 구조 → 시뮬레이션 관절각 변환)
        const double d = data[i];
        data[i] = 0.0695044662 * d * d * d - 0.0249173454 * d * d -
          0.786456081 * d + 1.5443387652 - M_PI / 2.0;
      }
      if (pub_controller) {
        std_msgs::msg::Float64 m;
        m.data = data[i];
        joint_controller_pubs_[i]->publish(m);
      }
    }
    if (pub_joint) {
      joint_state_.position = data;
      joint_state_pub_->publish(joint_state_);
    }
  }

  static bool str_true(const std::string & s)
  {
    std::string lower = s;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    return lower == "true";
  }

  // ── 상태 ──
  std::unique_ptr<pc::PuppyEngine> engine_;
  std::unique_ptr<Mpu6050> mpu_;
  std::map<std::string, PosePreset> pose_presets_;
  std::map<std::string, GaitPreset> gait_presets_;
  PosePreset cur_pose_;   // 원본의 global PuppyPose
  GaitPreset cur_gait_;   // 원본의 global GaitConfig
  int times_{0};
  std::string joint_state_pub_topic_{"false"};
  std::string joint_state_controller_pub_topic_{"false"};

  sensor_msgs::msg::JointState joint_state_;

  // ── ROS 인터페이스 ──
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_running_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_go_home_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_self_balancing_;
  rclcpp::Service<SetRunActionName>::SharedPtr srv_run_action_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_mark_time_;

  rclcpp::Subscription<GaitMsg>::SharedPtr sub_gait_;
  rclcpp::Subscription<VelocityMsg>::SharedPtr sub_velocity_;
  rclcpp::Subscription<VelocityMsg>::SharedPtr sub_velocity_move_;
  rclcpp::Subscription<VelocityMsg>::SharedPtr sub_velocity_autogait_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_nav_;
  rclcpp::Subscription<PoseMsg>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr sub_four_legs_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_gait_pc_;
  rclcpp::Subscription<SetServoMsg>::SharedPtr sub_set_servo_;

  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr legs_coord_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_controller_pubs_;
  rclcpp::Publisher<BuzzerState>::SharedPtr buzzer_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PuppyControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
