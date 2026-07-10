// ros_robot_controller_node.cpp
// ─────────────────────────────────────────────────────────────────────────────
// ros_robot_controller_node.py (287줄)의 C++ 이식판.
// 토픽/서비스 이름, 메시지 스케일링(IMU ×9.80665, gyro rad 변환), 공분산,
// 50Hz 발행 주기를 원본과 동일하게 유지 — 파이썬 노드의 드롭인 교체.
//
// 원본과 의도적으로 다른 점:
//  - 원본의 get_bus_servo_state 는 존재하지 않는 SDK 메서드
//    (bus_servo_read_voltage / bus_servo_read_torque)를 호출하는 버그가 있어
//    get_voltage / get_torque_state 요청 시 예외가 났음 → 올바른 메서드
//    (read_vin / read_torque_state)로 연결해 수정
//  - 서보 읽기 응답 대기에 1초 타임아웃 추가 (원본은 무한 대기 → 서비스 멈춤 가능)
// ─────────────────────────────────────────────────────────────────────────────

#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros_robot_controller_cpp/board.hpp"
#include "ros_robot_controller_msgs/msg/button_state.hpp"
#include "ros_robot_controller_msgs/msg/bus_servo_state.hpp"
#include "ros_robot_controller_msgs/msg/buzzer_state.hpp"
#include "ros_robot_controller_msgs/msg/led_state.hpp"
#include "ros_robot_controller_msgs/msg/motors_state.hpp"
#include "ros_robot_controller_msgs/msg/oled_state.hpp"
#include "ros_robot_controller_msgs/msg/pwm_servo_state.hpp"
#include "ros_robot_controller_msgs/msg/rg_bs_state.hpp"
#include "ros_robot_controller_msgs/msg/sbus.hpp"
#include "ros_robot_controller_msgs/msg/servos_position.hpp"
#include "ros_robot_controller_msgs/msg/set_bus_servo_state.hpp"
#include "ros_robot_controller_msgs/msg/set_pwm_servo_state.hpp"
#include "ros_robot_controller_msgs/srv/get_bus_servo_state.hpp"
#include "ros_robot_controller_msgs/srv/get_pwm_servo_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace rrc = ros_robot_controller_cpp;
namespace msgs = ros_robot_controller_msgs::msg;
namespace srvs = ros_robot_controller_msgs::srv;

class RosRobotController : public rclcpp::Node
{
public:
  static constexpr double kGravity = 9.80665;

  RosRobotController()
  : Node("ros_robot_controller")
  {
    board_ = std::make_unique<rrc::Board>();
    board_->enable_reception(true);

    declare_parameter<std::string>("imu_frame", "imu_link");
    declare_parameter<bool>("init_finish", false);
    imu_frame_ = get_parameter("imu_frame").as_string();

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("~/imu_raw", 1);
    joy_pub_ = create_publisher<sensor_msgs::msg::Joy>("~/joy", 1);
    sbus_pub_ = create_publisher<msgs::Sbus>("~/sbus", 1);
    button_pub_ = create_publisher<msgs::ButtonState>("~/button", 1);
    battery_pub_ = create_publisher<std_msgs::msg::UInt16>("~/battery", 1);

    sub_led_ = create_subscription<msgs::LedState>(
      "~/set_led", 5, [this](msgs::LedState::SharedPtr msg) {
        board_->set_led(msg->on_time, msg->off_time, msg->repeat, msg->id);
      });
    sub_buzzer_ = create_subscription<msgs::BuzzerState>(
      "~/set_buzzer", 5, [this](msgs::BuzzerState::SharedPtr msg) {
        board_->set_buzzer(msg->freq, msg->on_time, msg->off_time, msg->repeat);
      });
    sub_oled_ = create_subscription<msgs::OLEDState>(
      "~/set_oled", 5, [this](msgs::OLEDState::SharedPtr msg) {
        board_->set_oled_text(msg->index, msg->text);
      });
    sub_motor_ = create_subscription<msgs::MotorsState>(
      "~/set_motor", 10, [this](msgs::MotorsState::SharedPtr msg) {
        std::vector<std::pair<int, double>> data;
        for (const auto & m : msg->data) {
          data.emplace_back(m.id, m.rps);
        }
        board_->set_motor_speed(data);
      });
    sub_rgb_ = create_subscription<msgs::RGBsState>(
      "~/set_rgb", 10, [this](msgs::RGBsState::SharedPtr msg) {
        for (const auto & p : msg->data) {
          board_->set_rgb({{{p.id, p.r, p.g, p.b}}});
        }
      });
    sub_enable_ = create_subscription<std_msgs::msg::Bool>(
      "~/enable_reception", 1, [this](std_msgs::msg::Bool::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "enable_reception %s", msg->data ? "true" : "false");
        publish_enabled_ = msg->data;
        board_->enable_reception(msg->data);
      });
    sub_bus_state_ = create_subscription<msgs::SetBusServoState>(
      "~/bus_servo/set_state", 10,
      std::bind(&RosRobotController::set_bus_servo_state, this, std::placeholders::_1));
    sub_bus_pos_ = create_subscription<msgs::ServosPosition>(
      "~/bus_servo/set_position", 10, [this](msgs::ServosPosition::SharedPtr msg) {
        std::vector<std::pair<int, int>> data;
        for (const auto & p : msg->position) {
          data.emplace_back(p.id, p.position);
        }
        if (!data.empty()) {
          board_->bus_servo_set_position(msg->duration, data);
        }
      });
    sub_pwm_state_ = create_subscription<msgs::SetPWMServoState>(
      "~/pwm_servo/set_state", 10, [this](msgs::SetPWMServoState::SharedPtr msg) {
        std::vector<std::pair<int, int>> data;
        for (const auto & s : msg->state) {
          if (!s.id.empty() && !s.position.empty()) {
            data.emplace_back(s.id[0], s.position[0]);
          }
          if (!s.id.empty() && !s.offset.empty()) {
            board_->pwm_servo_set_offset(s.id[0], s.offset[0]);
          }
        }
        if (!data.empty()) {
          board_->pwm_servo_set_position(msg->duration, data);
        }
      });

    srv_bus_get_ = create_service<srvs::GetBusServoState>(
      "~/bus_servo/get_state",
      std::bind(&RosRobotController::get_bus_servo_state, this,
        std::placeholders::_1, std::placeholders::_2));
    srv_pwm_get_ = create_service<srvs::GetPWMServoState>(
      "~/pwm_servo/get_state",
      std::bind(&RosRobotController::get_pwm_servo_state, this,
        std::placeholders::_1, std::placeholders::_2));
    srv_init_finish_ = create_service<std_srvs::srv::Trigger>(
      "~/init_finish",
      [](std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = true;
      });

    // 원본 초기화 시퀀스: pwm 오프셋 리셋 + 모터 정지
    board_->pwm_servo_set_offset(1, 0);
    board_->set_motor_speed({{1, 0}, {2, 0}, {3, 0}, {4, 0}});

    pub_thread_ = std::thread(&RosRobotController::pub_loop, this);
    RCLCPP_INFO(get_logger(), "\033[1;32mstart\033[0m");
  }

  ~RosRobotController() override
  {
    running_ = false;
    if (pub_thread_.joinable()) {
      pub_thread_.join();
    }
    // 종료 시 모터 정지 (원본 KeyboardInterrupt 처리와 동일)
    board_->set_motor_speed({{1, 0}, {2, 0}, {3, 0}, {4, 0}});
  }

private:
  // 원본 pub_callback: 50Hz 로 보드 상태를 ROS 토픽으로 발행
  void pub_loop()
  {
    while (running_ && rclcpp::ok()) {
      if (publish_enabled_) {
        publish_button();
        publish_joy();
        publish_imu();
        publish_sbus();
        publish_battery();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  void publish_battery()
  {
    if (auto v = board_->get_battery()) {
      std_msgs::msg::UInt16 msg;
      msg.data = *v;
      battery_pub_->publish(msg);
    }
  }

  void publish_button()
  {
    if (auto v = board_->get_button()) {
      msgs::ButtonState msg;
      msg.id = v->first;
      msg.state = v->second;
      button_pub_->publish(msg);
    }
  }

  void publish_joy()
  {
    if (auto v = board_->get_gamepad()) {
      sensor_msgs::msg::Joy msg;
      msg.axes.assign(v->axes.begin(), v->axes.end());
      msg.buttons.assign(v->buttons.begin(), v->buttons.end());
      msg.header.stamp = get_clock()->now();
      joy_pub_->publish(msg);
    }
  }

  void publish_sbus()
  {
    if (auto v = board_->get_sbus()) {
      msgs::Sbus msg;
      msg.channel = *v;
      msg.header.stamp = get_clock()->now();
      sbus_pub_->publish(msg);
    }
  }

  void publish_imu()
  {
    if (auto v = board_->get_imu()) {
      const auto & d = *v;  // ax, ay, az, gx, gy, gz
      sensor_msgs::msg::Imu msg;
      msg.header.frame_id = imu_frame_;
      msg.header.stamp = get_clock()->now();

      msg.linear_acceleration.x = d[0] * kGravity;
      msg.linear_acceleration.y = d[1] * kGravity;
      msg.linear_acceleration.z = d[2] * kGravity;
      msg.angular_velocity.x = d[3] * M_PI / 180.0;
      msg.angular_velocity.y = d[4] * M_PI / 180.0;
      msg.angular_velocity.z = d[5] * M_PI / 180.0;

      msg.orientation_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
      msg.angular_velocity_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
      msg.linear_acceleration_covariance = {0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.004};
      imu_pub_->publish(msg);
    }
  }

  // 원본 set_bus_servo_state 의 [0]=플래그, [1]=값 배열 규약을 그대로 유지
  void set_bus_servo_state(msgs::SetBusServoState::SharedPtr msg)
  {
    std::vector<std::pair<int, int>> data;
    std::vector<int> stop_ids;
    for (const auto & s : msg->state) {
      if (s.present_id.empty() || !s.present_id[0]) {
        continue;
      }
      const int id = s.present_id.size() > 1 ? s.present_id[1] : 0;
      if (s.target_id.size() > 1 && s.target_id[0]) {
        board_->bus_servo_set_id(id, s.target_id[1]);
      }
      if (s.position.size() > 1 && s.position[0]) {
        data.emplace_back(id, s.position[1]);
      }
      if (s.offset.size() > 1 && s.offset[0]) {
        board_->bus_servo_set_offset(id, s.offset[1]);
      }
      if (s.position_limit.size() > 2 && s.position_limit[0]) {
        board_->bus_servo_set_angle_limit(id, s.position_limit[1], s.position_limit[2]);
      }
      if (s.voltage_limit.size() > 2 && s.voltage_limit[0]) {
        board_->bus_servo_set_vin_limit(id, s.voltage_limit[1], s.voltage_limit[2]);
      }
      if (s.max_temperature_limit.size() > 1 && s.max_temperature_limit[0]) {
        board_->bus_servo_set_temp_limit(id, s.max_temperature_limit[1]);
      }
      if (s.enable_torque.size() > 1 && s.enable_torque[0]) {
        board_->bus_servo_enable_torque(id, s.enable_torque[1] != 0);
      }
      if (!s.save_offset.empty() && s.save_offset[0]) {
        board_->bus_servo_save_offset(id);
      }
      if (!s.stop.empty() && s.stop[0]) {
        stop_ids.push_back(id);
      }
    }
    if (!data.empty()) {
      board_->bus_servo_set_position(msg->duration, data);
    }
    if (!stop_ids.empty()) {
      board_->bus_servo_stop(stop_ids);
    }
  }

  void get_bus_servo_state(
    srvs::GetBusServoState::Request::SharedPtr request,
    srvs::GetBusServoState::Response::SharedPtr response)
  {
    for (auto & cmd : request->cmd) {
      msgs::BusServoState st;
      int id = cmd.id;
      if (cmd.get_id) {
        if (auto v = board_->bus_servo_read_id(id)) {
          id = *v;
          cmd.id = static_cast<uint8_t>(*v);
          st.present_id = {static_cast<uint16_t>(*v)};
        }
      }
      if (cmd.get_position) {
        if (auto v = board_->bus_servo_read_position(id)) {
          st.position = {static_cast<uint16_t>(*v)};
        }
      }
      if (cmd.get_offset) {
        if (auto v = board_->bus_servo_read_offset(id)) {
          st.offset = {static_cast<int16_t>(*v)};
        }
      }
      if (cmd.get_voltage) {
        if (auto v = board_->bus_servo_read_vin(id)) {
          st.voltage = {static_cast<uint16_t>(*v)};
        }
      }
      if (cmd.get_temperature) {
        if (auto v = board_->bus_servo_read_temp(id)) {
          st.temperature = {static_cast<uint16_t>(*v)};
        }
      }
      if (cmd.get_position_limit) {
        if (auto v = board_->bus_servo_read_angle_limit(id)) {
          st.position_limit = {static_cast<uint16_t>(v->first), static_cast<uint16_t>(v->second)};
        }
      }
      if (cmd.get_voltage_limit) {
        if (auto v = board_->bus_servo_read_vin_limit(id)) {
          st.voltage_limit = {static_cast<uint16_t>(v->first), static_cast<uint16_t>(v->second)};
        }
      }
      if (cmd.get_max_temperature_limit) {
        if (auto v = board_->bus_servo_read_temp_limit(id)) {
          st.max_temperature_limit = {static_cast<uint16_t>(*v)};
        }
      }
      if (cmd.get_torque_state) {
        if (auto v = board_->bus_servo_read_torque_state(id)) {
          st.enable_torque = {static_cast<uint16_t>(*v)};
        }
      }
      response->state.push_back(st);
    }
    response->success = true;
  }

  void get_pwm_servo_state(
    srvs::GetPWMServoState::Request::SharedPtr request,
    srvs::GetPWMServoState::Response::SharedPtr response)
  {
    for (const auto & cmd : request->cmd) {
      msgs::PWMServoState st;
      if (cmd.get_position) {
        if (auto v = board_->pwm_servo_read_position(cmd.id)) {
          st.position = {static_cast<uint16_t>(*v)};
        }
      }
      if (cmd.get_offset) {
        if (auto v = board_->pwm_servo_read_offset(cmd.id)) {
          st.offset = {static_cast<int16_t>(*v)};
        }
      }
      response->state.push_back(st);
    }
    response->success = true;
  }

  std::unique_ptr<rrc::Board> board_;
  std::string imu_frame_;
  std::atomic<bool> running_{true};
  std::atomic<bool> publish_enabled_{true};
  std::thread pub_thread_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Publisher<msgs::Sbus>::SharedPtr sbus_pub_;
  rclcpp::Publisher<msgs::ButtonState>::SharedPtr button_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr battery_pub_;

  rclcpp::Subscription<msgs::LedState>::SharedPtr sub_led_;
  rclcpp::Subscription<msgs::BuzzerState>::SharedPtr sub_buzzer_;
  rclcpp::Subscription<msgs::OLEDState>::SharedPtr sub_oled_;
  rclcpp::Subscription<msgs::MotorsState>::SharedPtr sub_motor_;
  rclcpp::Subscription<msgs::RGBsState>::SharedPtr sub_rgb_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
  rclcpp::Subscription<msgs::SetBusServoState>::SharedPtr sub_bus_state_;
  rclcpp::Subscription<msgs::ServosPosition>::SharedPtr sub_bus_pos_;
  rclcpp::Subscription<msgs::SetPWMServoState>::SharedPtr sub_pwm_state_;

  rclcpp::Service<srvs::GetBusServoState>::SharedPtr srv_bus_get_;
  rclcpp::Service<srvs::GetPWMServoState>::SharedPtr srv_pwm_get_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_init_finish_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RosRobotController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
