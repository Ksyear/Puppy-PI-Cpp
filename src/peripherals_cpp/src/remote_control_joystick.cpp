// remote_control_joystick.cpp
// ─────────────────────────────────────────────────────────────────────────────
// remote_control_joystick.py (353줄)의 C++ 이식판 — 무선 게임패드로 PuppyPi 조종.
//
// 원본은 pygame 으로 /dev/input/js0 을 읽지만, C++ 에서는 리눅스 조이스틱 API
// (linux/joystick.h 의 js_event)를 직접 읽는다. 나머지(버튼 배열 순서, 아날로그→
// 디지털 변환 임계값 ±0.5, 3틱 후 HOLD 판정, 속도/자세 클램프, 발행 토픽)는
// 원본과 동일하다.
//
// ★ 실기기 검증 필요: pygame 과 리눅스 js API 의 축/버튼 번호 매핑은 장치에 따라
//   다를 수 있다. hat 이 축 4,5 가 아니거나 방향이 반대면 파라미터
//   (hat_x_axis, hat_y_axis, invert_hat_y)로 보정할 것.
// ─────────────────────────────────────────────────────────────────────────────

#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/polygon.hpp"
#include "puppy_control_msgs/msg/pose.hpp"
#include "puppy_control_msgs/msg/velocity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_robot_controller_msgs/msg/buzzer_state.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;
using Velocity = puppy_control_msgs::msg::Velocity;
using PoseMsg = puppy_control_msgs::msg::Pose;
using BuzzerState = ros_robot_controller_msgs::msg::BuzzerState;

namespace
{

inline double radians(double deg) {return deg * M_PI / 180.0;}

// 원본 BUTTONS 튜플과 동일한 순서 (인덱스가 곧 의미)
enum Btn : int
{
  CROSS = 0, CIRCLE, NONE_1, SQUARE, TRIANGLE, NONE_2, L1, R1,
  L2, R2, SELECT, START, MODE,
  L_HAT_LEFT, L_HAT_RIGHT, L_HAT_DOWN, L_HAT_UP,
  L_AXIS_LEFT, L_AXIS_RIGHT, L_AXIS_UP, L_AXIS_DOWN,
  R_AXIS_LEFT, R_AXIS_RIGHT, R_AXIS_UP, R_AXIS_DOWN,
  BTN_COUNT  // = 25
};

// 원본 Stand 자세 (remote_control_joystick.py 상단 상수)
struct PoseState
{
  double roll{0.0}, pitch{0.0}, yaw{0.0};
  double height{-10.0}, x_shift{-0.5};
  double stance_x{0.0}, stance_y{0.0};
};

}  // namespace

// /dev/input/js0 을 읽는 클래스 (원본 Joystick 클래스 대응, pygame 대체)
class Joystick
{
public:
  Joystick(const std::string & device, int hat_x_axis, int hat_y_axis, bool invert_hat_y)
  : device_(device), hat_x_axis_(hat_x_axis), hat_y_axis_(hat_y_axis),
    invert_hat_y_(invert_hat_y)
  {
    thread_ = std::thread(&Joystick::loop, this);
  }

  ~Joystick()
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  bool connected() const {return fd_ >= 0;}

  // 원본 update_buttons 의 데이터 수집부: 13개 버튼 + 6개 아날로그(hat 2 + 스틱 4)를
  // 25개 디지털 상태로 변환 (아날로그는 < -0.5 / > 0.5 로 두 개의 디지털이 됨)
  std::array<int, BTN_COUNT> digital_state()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::array<int, BTN_COUNT> out{};
    for (int i = 0; i < 13; ++i) {
      out[i] = buttons_[i];
    }
    const double hat_x = axes_[hat_x_axis_];
    const double hat_y = (invert_hat_y_ ? -1.0 : 1.0) * axes_[hat_y_axis_];
    const double analog[6] = {hat_x, hat_y, axes_[0], axes_[1], axes_[2], axes_[3]};
    for (int i = 0; i < 6; ++i) {
      out[13 + i * 2] = analog[i] < -0.5 ? 1 : 0;
      out[13 + i * 2 + 1] = analog[i] > 0.5 ? 1 : 0;
    }
    return out;
  }

private:
  // 원본 connect(): 0.2초 주기로 연결/해제 감시 + 이벤트 읽기
  void loop()
  {
    while (running_) {
      if (fd_ < 0) {
        fd_ = ::open(device_.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ < 0) {
          std::this_thread::sleep_for(200ms);
          continue;
        }
      }
      js_event ev;
      while (true) {
        const ssize_t n = ::read(fd_, &ev, sizeof(ev));
        if (n != sizeof(ev)) {
          if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            break;  // 대기 중인 이벤트 없음
          }
          ::close(fd_);   // 장치 분리 등 → 재연결 시도
          fd_ = -1;
          break;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        const int type = ev.type & ~JS_EVENT_INIT;
        if (type == JS_EVENT_BUTTON && ev.number < buttons_.size()) {
          buttons_[ev.number] = ev.value ? 1 : 0;
        } else if (type == JS_EVENT_AXIS && ev.number < axes_.size()) {
          axes_[ev.number] = ev.value / 32767.0;
        }
      }
      std::this_thread::sleep_for(10ms);
    }
  }

  std::string device_;
  int hat_x_axis_;
  int hat_y_axis_;
  bool invert_hat_y_;
  int fd_{-1};
  std::atomic<bool> running_{true};
  std::thread thread_;
  std::mutex mutex_;
  std::array<int, 16> buttons_{};
  std::array<double, 16> axes_{};
};

class RemoteControlNode : public rclcpp::Node
{
public:
  RemoteControlNode()
  : Node("remote_control_joystick")
  {
    const auto device = declare_parameter<std::string>("device", "/dev/input/js0");
    const int hat_x_axis = declare_parameter<int>("hat_x_axis", 4);
    const int hat_y_axis = declare_parameter<int>("hat_y_axis", 5);
    const bool invert_hat_y = declare_parameter<bool>("invert_hat_y", true);

    buzzer_pub_ = create_publisher<BuzzerState>("/ros_robot_controller/set_buzzer", 1);
    velocity_pub_ = create_publisher<Velocity>("/puppy_control/velocity/autogait", 1);
    pose_pub_ = create_publisher<PoseMsg>("/puppy_control/pose", 1);
    legs_coord_sub_ = create_subscription<geometry_msgs::msg::Polygon>(
      "/puppy_control/legs_coord", 1,
      [this](geometry_msgs::msg::Polygon::SharedPtr msg) {legs_coord_ = *msg;});
    go_home_client_ = create_client<std_srvs::srv::Empty>("/puppy_control/go_home");

    js_ = std::make_unique<Joystick>(device, hat_x_axis, hat_y_axis, invert_hat_y);
    timer_ = create_wall_timer(50ms, std::bind(&RemoteControlNode::update_buttons, this));
  }

private:
  // ── 원본 update_buttons: 변화 감지(xor) + PRESSED/HOLD/RELEASED 디스패치 ──
  void update_buttons()
  {
    if (!js_->connected()) {
      return;
    }
    const auto buttons = js_->digital_state();
    for (int i = 0; i < BTN_COUNT; ++i) {
      if (buttons[i] != last_buttons_[i]) {
        if (buttons[i]) {
          hold_count_[i] = 0;
          on_pressed(static_cast<Btn>(i));
        } else {
          on_released(static_cast<Btn>(i));
        }
      } else if (buttons[i]) {
        // 3틱(0.15초) 이상 눌려 있으면 HOLD 로 판정 — 원본과 동일
        if (hold_count_[i] < 3) {
          ++hold_count_[i];
        } else {
          on_hold(static_cast<Btn>(i));
        }
      }
    }
    last_buttons_ = buttons;
  }

  bool up_down_held() const
  {
    return last_buttons_[L_HAT_UP] || last_buttons_[L_AXIS_UP] ||
           last_buttons_[L_HAT_DOWN] || last_buttons_[L_AXIS_DOWN];
  }

  void publish_velocity()
  {
    Velocity msg;
    msg.x = static_cast<float>(move_x_);
    msg.y = 0.0f;
    msg.yaw_rate = static_cast<float>(move_yaw_);
    velocity_pub_->publish(msg);
  }

  void publish_pose()
  {
    PoseMsg msg;
    msg.roll = static_cast<float>(pose_.roll);
    msg.pitch = static_cast<float>(pose_.pitch);
    msg.yaw = static_cast<float>(pose_.yaw);
    msg.height = static_cast<float>(pose_.height);
    msg.x_shift = static_cast<float>(pose_.x_shift);
    msg.stance_x = static_cast<float>(pose_.stance_x);
    msg.stance_y = static_cast<float>(pose_.stance_y);
    pose_pub_->publish(msg);
  }

  // 원본 PressedFun 의 CIRCLE/SQUARE 속도 가감 블록
  void adjust_speed(double delta)
  {
    if (!up_down_held()) {
      return;
    }
    velocity_x_ = std::min(25.0, std::max(5.0, velocity_x_ + delta));
    if (move_x_ > 0) {
      move_x_ = velocity_x_;
    } else if (move_x_ < 0) {
      move_x_ = -velocity_x_;
    }
    publish_velocity();
    RCLCPP_INFO(get_logger(), "VelocityX=%.1f", velocity_x_);
  }

  // 원본 PressedFun 의 자세 키 블록 (클램프 동일)
  void adjust_pose(const char * key, double delta)
  {
    if (std::string(key) == "height") {
      pose_.height = std::min(-5.0, std::max(-16.0, pose_.height + delta));
    } else if (std::string(key) == "x_shift") {
      pose_.x_shift = std::min(6.0, std::max(-6.0, pose_.x_shift + delta));
    } else if (std::string(key) == "pitch") {
      pose_.pitch = std::min(radians(20.0), std::max(-radians(20.0), pose_.pitch + delta));
    }
    publish_pose();
  }

  void go_home()
  {
    RCLCPP_INFO(get_logger(), "go_home");
    pose_ = PoseState{};
    move_x_ = 0.0;
    move_yaw_ = 0.0;
    go_home_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());

    BuzzerState buzzer;
    buzzer.freq = 1900;
    buzzer.on_time = 0.1f;
    buzzer.off_time = 0.9f;
    buzzer.repeat = 1;
    buzzer_pub_->publish(buzzer);
  }

  void on_pressed(Btn b)
  {
    switch (b) {
      case START: go_home(); break;
      case SELECT:
        RCLCPP_INFO(get_logger(), "legs_coord: %zu points", legs_coord_.points.size());
        break;
      case L_HAT_UP: case L_AXIS_UP:
        move_x_ = velocity_x_;
        publish_velocity();
        break;
      case L_HAT_DOWN: case L_AXIS_DOWN:
        move_x_ = -velocity_x_;
        publish_velocity();
        break;
      case L_HAT_LEFT: case L_AXIS_LEFT:
        move_yaw_ = radians(20.0);
        publish_velocity();
        break;
      case L_HAT_RIGHT: case L_AXIS_RIGHT:
        move_yaw_ = -radians(20.0);
        publish_velocity();
        break;
      case TRIANGLE: adjust_pose("height", -0.15); break;
      case CROSS: adjust_pose("height", 0.15); break;
      case CIRCLE: adjust_speed(1.0); break;
      case SQUARE: adjust_speed(-1.0); break;
      case L1: adjust_pose("pitch", 0.015); break;
      case L2: adjust_pose("pitch", -0.015); break;
      case R1: adjust_pose("x_shift", -0.15); break;
      case R2: adjust_pose("x_shift", 0.15); break;
      default: break;
    }
  }

  void on_hold(Btn b)
  {
    switch (b) {
      case TRIANGLE: adjust_pose("height", -0.15); break;
      case CROSS: adjust_pose("height", 0.15); break;
      case CIRCLE: adjust_speed(0.5); break;
      case SQUARE: adjust_speed(-0.5); break;
      case L1: adjust_pose("pitch", 0.015); break;
      case L2: adjust_pose("pitch", -0.015); break;
      case R1: adjust_pose("x_shift", -0.15); break;
      case R2: adjust_pose("x_shift", 0.15); break;
      default: break;
    }
  }

  void on_released(Btn b)
  {
    switch (b) {
      case L_HAT_UP: case L_AXIS_UP: case L_HAT_DOWN: case L_AXIS_DOWN:
        move_x_ = 0.0;
        publish_velocity();
        break;
      case L_HAT_LEFT: case L_AXIS_LEFT: case L_HAT_RIGHT: case L_AXIS_RIGHT:
        move_yaw_ = 0.0;
        publish_velocity();
        break;
      default: break;
    }
  }

  std::unique_ptr<Joystick> js_;
  std::array<int, BTN_COUNT> last_buttons_{};
  std::array<int, BTN_COUNT> hold_count_{};

  PoseState pose_;                 // 원본 PuppyPose (Stand 초기값)
  double move_x_{0.0};             // 원본 PuppyMove['x']
  double move_yaw_{0.0};           // 원본 PuppyMove['yaw_rate']
  double velocity_x_{15.0};        // 원본 VelocityX (5~25)
  geometry_msgs::msg::Polygon legs_coord_;

  rclcpp::Publisher<BuzzerState>::SharedPtr buzzer_pub_;
  rclcpp::Publisher<Velocity>::SharedPtr velocity_pub_;
  rclcpp::Publisher<PoseMsg>::SharedPtr pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr legs_coord_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr go_home_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RemoteControlNode>());
  rclcpp::shutdown();
  return 0;
}
