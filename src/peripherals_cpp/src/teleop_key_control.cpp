// teleop_key_control.cpp
// ─────────────────────────────────────────────────────────────────────────────
// teleop_key_control.py (150줄)의 C++ 이식판 — 키보드(w/a/s/d)로 Twist 발행.
// 터미널을 raw 모드로 바꿔 키를 즉시 읽고, controller/cmd_vel 로 발행한다.
// MACHINE_TYPE=JetRover_Acker 분기(애커만 조향)도 원본과 동일하게 유지.
// ─────────────────────────────────────────────────────────────────────────────

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

termios g_saved_settings;

// 원본 getKey(): 0.1초 동안 키 입력을 기다렸다가 없으면 빈 문자 반환
char get_key()
{
  termios raw = g_saved_settings;
  ::cfmakeraw(&raw);
  ::tcsetattr(STDIN_FILENO, TCSANOW, &raw);

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  timeval tv{0, 100000};  // 0.1s

  char key = '\0';
  if (::select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0) {
    if (::read(STDIN_FILENO, &key, 1) != 1) {
      key = '\0';
    }
  }
  ::tcsetattr(STDIN_FILENO, TCSADRAIN, &g_saved_settings);
  return key;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("teleop_control");
  auto cmd_vel = node->create_publisher<geometry_msgs::msg::Twist>("controller/cmd_vel", 1);

  ::tcgetattr(STDIN_FILENO, &g_saved_settings);

  const char * machine_type_env = std::getenv("MACHINE_TYPE");
  const bool is_acker =
    machine_type_env && std::string(machine_type_env) == "JetRover_Acker";
  const double LIN_VEL = 0.2;
  const double ANG_VEL = is_acker ? LIN_VEL / (0.213 / std::tan(0.628)) : 0.5;

  printf(
    "\nControl Your Robot!\n---------------------------\nMoving around:\n"
    "        w\n   a    s    d\nCTRL-C to quit\n\n");

  double linear = 0.0;
  double angular = 0.0;
  double last_x = 0.0;
  double last_z = 0.0;
  int count = 0;

  while (rclcpp::ok()) {
    const char key = get_key();
    if (!is_acker) {
      if (key == 'w') {
        linear = LIN_VEL;
      } else if (key == 'a') {
        angular = ANG_VEL;
        linear = 0.0;
      } else if (key == 'd') {
        angular = -ANG_VEL;
        linear = 0.0;
      } else if (key == 's') {
        linear = -LIN_VEL;
      } else if (key == '\0') {
        angular = 0.0;
      } else if (key == '\x03') {  // Ctrl+C
        break;
      }
    } else {
      if (key == 'w') {
        count = 0;
        linear = LIN_VEL;
      } else if (key == 'a') {
        count = 0;
        angular = ANG_VEL;
      } else if (key == 'd') {
        count = 0;
        angular = -ANG_VEL;
      } else if (key == 's') {
        count = 0;
        linear = -LIN_VEL;
      } else if (key == '\0') {
        if (++count > 5) {
          count = 0;
          if (angular != 0.0) {
            angular = 0.0;
            linear = 0.0;
          }
        }
      } else {
        count = 0;
        if (key == '\x03') {
          break;
        }
      }
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear;
    twist.angular.z = angular;
    // 원본과 동일: 값이 바뀌었거나 회전 중일 때만 발행
    if (last_x != linear || last_z != angular || angular != 0.0) {
      cmd_vel->publish(twist);
    }
    last_x = linear;
    last_z = angular;
  }

  cmd_vel->publish(geometry_msgs::msg::Twist());  // 종료 시 정지
  ::tcsetattr(STDIN_FILENO, TCSADRAIN, &g_saved_settings);
  rclcpp::shutdown();
  return 0;
}
