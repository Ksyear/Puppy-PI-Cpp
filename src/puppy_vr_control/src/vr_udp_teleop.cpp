// vr_udp_teleop.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Meta Quest(Unity)의 UDP_Joystick_Sender.cs 가 보내는 조이스틱 기울기 각도
// ("X:12.3,Z:-5.0" 형식, 기본 20Hz)를 UDP로 수신해서
// puppy_control_msgs/Velocity 메시지로 변환해 /puppy_control/velocity/autogait 로 발행한다.
//
// PuppyPi_MR_Controller 저장소의 raspberry_pi/udp_joystick_receiver.py 의 C++/ROS2 이식판.
// 원본 파이썬의 RobotDriver.drive()/stop() 은 비어 있었는데(주석 예시만 존재),
// 이 노드는 그 부분을 PuppyPi 공식 조이스틱 노드(remote_control_joystick.py)와
// 동일한 토픽/단위로 실제 구현한 것이다.
//
// 변환 규칙 (원본 파이썬과 동일):
//   - 데드존: |각도| < deadzone_deg 이면 0 (손떨림/노이즈 무시)
//   - 정규화: 각도 / max_angle_deg 를 -1.0 ~ 1.0 으로 클램프
//   - Z(앞뒤 기울기) → 전진/후진,  X(좌우 기울기) → 회전
//   - recv_timeout_sec 동안 패킷이 없으면 안전 정지
//
// PuppyPi 단위계 (puppy.py 참고):
//   - Velocity.x  : 전진 속도, cm/s 스케일 (|x| <= 35 넘으면 puppy_control이 무시)
//   - Velocity.y  : 반드시 0 (0이 아니면 puppy_control이 무시)
//   - Velocity.yaw_rate : 회전 속도, rad/s (|yaw| <= radians(51) 넘으면 무시)
//   - x == 0, y == 0, yaw_rate == 0 이면 정지 (move_stop)
// ─────────────────────────────────────────────────────────────────────────────

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "puppy_control_msgs/msg/velocity.hpp"
#include "puppy_vr_control/vr_teleop_logic.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;
using Velocity = puppy_control_msgs::msg::Velocity;
using puppy_vr_control::parse_packet;   // 순수 로직은 vr_teleop_logic.hpp 로 분리
                                        // (colcon test 로 단독 검증됨)

class VrUdpTeleop : public rclcpp::Node
{
public:
  VrUdpTeleop()
  : Node("vr_udp_teleop")
  {
    // ── 파라미터 선언 (config/vr_control_params.yaml 로 덮어쓸 수 있음) ──
    port_ = declare_parameter<int>("port", 5005);
    deadzone_deg_ = declare_parameter<double>("deadzone_deg", 5.0);
    max_angle_deg_ = declare_parameter<double>("max_angle_deg", 45.0);
    recv_timeout_sec_ = declare_parameter<double>("recv_timeout_sec", 1.0);

    // 최대 속도: 공식 조이스틱 노드의 기본값(VelocityX=15, yaw ±20°)과 동일
    max_speed_x_ = declare_parameter<double>("max_speed_x", 15.0);          // cm/s
    max_yaw_rate_deg_ = declare_parameter<double>("max_yaw_rate_deg", 20.0);  // deg/s

    // 실기기에서 방향이 반대로 움직이면 이 두 개를 뒤집으면 된다
    invert_forward_ = declare_parameter<bool>("invert_forward", false);
    invert_turn_ = declare_parameter<bool>("invert_turn", true);

    velocity_topic_ = declare_parameter<std::string>(
      "velocity_topic", "/puppy_control/velocity/autogait");
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);

    // 같은 값을 계속 다시 보내 gait_config가 매번 재계산되는 것을 막기 위한 양자화 폭
    speed_step_ = declare_parameter<double>("speed_step", 1.0);          // cm/s 단위
    yaw_step_deg_ = declare_parameter<double>("yaw_step_deg", 2.0);      // deg/s 단위
    heartbeat_sec_ = declare_parameter<double>("heartbeat_sec", 0.5);

    debug_ = declare_parameter<bool>("debug", false);

    // puppy_control 이 조용히 무시하는 범위를 넘지 않도록 상한을 강제
    if (max_speed_x_ > 35.0) {
      RCLCPP_WARN(get_logger(), "max_speed_x %.1f > 35 (puppy_control 허용 한계) → 35로 제한", max_speed_x_);
      max_speed_x_ = 35.0;
    }
    if (max_yaw_rate_deg_ > 51.0) {
      RCLCPP_WARN(get_logger(), "max_yaw_rate_deg %.1f > 51 (허용 한계) → 51로 제한", max_yaw_rate_deg_);
      max_yaw_rate_deg_ = 51.0;
    }

    velocity_pub_ = create_publisher<Velocity>(velocity_topic_, 1);

    open_socket();
    rx_thread_ = std::thread(&VrUdpTeleop::rx_loop, this);

    const auto period =
      std::chrono::duration<double>(1.0 / std::max(publish_rate_, 1.0));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&VrUdpTeleop::control_tick, this));

    // 시작 시 서기 자세 (go_home) — 엎드린 채 시작하지 않도록
    if (declare_parameter<bool>("stand_on_start", true)) {
      go_home_client_ = create_client<std_srvs::srv::Empty>("/puppy_control/go_home");
      stand_timer_ = create_wall_timer(
        1s, [this]() {
          if (go_home_client_->service_is_ready()) {
            go_home_client_->async_send_request(
              std::make_shared<std_srvs::srv::Empty::Request>());
            RCLCPP_INFO(get_logger(), "시작 자세: go_home(서기) 호출");
            stand_timer_->cancel();
          } else if (++stand_tries_ > 15) {
            RCLCPP_WARN(get_logger(), "go_home 서비스 없음 — 서기 자세 생략");
            stand_timer_->cancel();
          }
        });
    }

    RCLCPP_INFO(
      get_logger(),
      "VR UDP 조이스틱 수신 대기: 0.0.0.0:%d  (데드존 ±%.1f°, 최대각 %.1f°, 무신호정지 %.1fs)",
      port_, deadzone_deg_, max_angle_deg_, recv_timeout_sec_);
    RCLCPP_INFO(
      get_logger(), "발행 토픽: %s  (최대 x=%.1f cm/s, yaw=%.1f deg/s)",
      velocity_topic_.c_str(), max_speed_x_, max_yaw_rate_deg_);
  }

  ~VrUdpTeleop() override
  {
    running_ = false;
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
    // 종료 시 안전 정지 명령을 한 번 보낸다
    if (velocity_pub_) {
      velocity_pub_->publish(Velocity());
    }
    if (sock_fd_ >= 0) {
      ::close(sock_fd_);
    }
  }

private:
  void open_socket()
  {
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
      throw std::runtime_error("UDP 소켓 생성 실패");
    }
    const int reuse = 1;
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);  // 0.0.0.0 : 모든 인터페이스에서 수신
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (::bind(sock_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      ::close(sock_fd_);
      sock_fd_ = -1;
      throw std::runtime_error(
        "UDP 포트 " + std::to_string(port_) + " 바인드 실패 (이미 사용 중인지 확인)");
    }
  }

  // 수신 스레드: 패킷을 모두 비우고 가장 최근의 유효한 값만 저장한다.
  // (오래된 패킷이 쌓여서 지연되는 것을 막기 위해 drain-latest 방식 사용)
  void rx_loop()
  {
    char buf[1024];
    while (running_ && rclcpp::ok()) {
      pollfd pfd{sock_fd_, POLLIN, 0};
      const int rc = ::poll(&pfd, 1, 100);  // 100ms 마다 종료 플래그 확인
      if (rc <= 0 || !(pfd.revents & POLLIN)) {
        continue;
      }

      std::optional<std::pair<float, float>> latest;
      sockaddr_in src{};
      socklen_t src_len = sizeof(src);
      while (true) {
        const ssize_t n = ::recvfrom(
          sock_fd_, buf, sizeof(buf) - 1, MSG_DONTWAIT,
          reinterpret_cast<sockaddr *>(&src), &src_len);
        if (n <= 0) {
          break;  // 더 읽을 패킷 없음
        }
        buf[n] = '\0';
        const std::string text(buf, static_cast<size_t>(n));
        // 긴급정지 프로토콜: "ESTOP" 수신 → 즉시 정지 + RESUME 까지 명령 무시
        if (text.find("ESTOP") != std::string::npos) {
          if (!estop_) {
            RCLCPP_ERROR(get_logger(), "[ESTOP] 긴급 정지 수신 — RESUME 전까지 모든 이동 명령 무시");
          }
          estop_ = true;
          continue;
        }
        if (text.find("RESUME") != std::string::npos) {
          if (estop_) {
            RCLCPP_WARN(get_logger(), "[RESUME] 긴급 정지 해제");
          }
          estop_ = false;
          continue;
        }
        const auto parsed = parse_packet(text);
        if (parsed) {
          latest = parsed;
        } else if (debug_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "[형식오류] 수신: '%s'", buf);
        }
      }

      if (latest) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_x_angle_ = latest->first;
        cmd_z_angle_ = latest->second;
        last_rx_ = std::chrono::steady_clock::now();
        have_cmd_ = true;

        if (debug_) {
          char ip[INET_ADDRSTRLEN] = "?";
          ::inet_ntop(AF_INET, &src.sin_addr, ip, sizeof(ip));
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[수신] %s  X=%6.1f Z=%6.1f", ip, cmd_x_angle_, cmd_z_angle_);
        }
      }
    }
  }

  double normalize(double angle) const
  {
    return puppy_vr_control::normalize_angle(angle, deadzone_deg_, max_angle_deg_);
  }

  // 주기적 제어 루프: 최신 명령을 Velocity 로 변환해 발행 + 무신호 안전 정지
  void control_tick()
  {
    float x_angle = 0.0f;
    float z_angle = 0.0f;
    bool have_cmd = false;
    std::chrono::steady_clock::time_point last_rx;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      x_angle = cmd_x_angle_;
      z_angle = cmd_z_angle_;
      have_cmd = have_cmd_;
      last_rx = last_rx_;
    }

    const auto now = std::chrono::steady_clock::now();
    const double age =
      std::chrono::duration<double>(now - last_rx).count();

    // ── 긴급 정지 상태: 정지 명령만 발행하고 이동 명령은 무시 ──
    if (estop_) {
      if (moving_) {
        publish_stop();
        moving_ = false;
        stop_repeat_ = 3;
      } else if (stop_repeat_ > 0) {
        publish_stop();
        --stop_repeat_;
      }
      return;
    }

    // ── 무신호 안전 정지: 일정 시간 패킷이 없으면 정지 명령 발행 ──
    if (!have_cmd || age > recv_timeout_sec_) {
      if (moving_) {
        publish_stop();
        moving_ = false;
        RCLCPP_WARN(get_logger(), "[무신호 %.1fs] 안전 정지", recv_timeout_sec_);
      } else if (stop_repeat_ > 0) {
        publish_stop();  // 정지 명령 유실 대비 몇 번 더 발행
        --stop_repeat_;
      }
      return;
    }

    // ── 각도 → 속도 변환 ──
    const double forward =
      normalize(z_angle) * (invert_forward_ ? -1.0 : 1.0);
    const double turn =
      normalize(x_angle) * (invert_turn_ ? -1.0 : 1.0);

    // 양자화: 스틱 미세 떨림으로 매 주기 값이 바뀌어
    // puppy_control 이 gait_config 를 계속 재계산하는 것을 막는다
    const double yaw_step = yaw_step_deg_ * M_PI / 180.0;
    const double max_yaw = max_yaw_rate_deg_ * M_PI / 180.0;
    double vx = std::round(forward * max_speed_x_ / speed_step_) * speed_step_;
    double vyaw = std::round(turn * max_yaw / yaw_step) * yaw_step;
    vx = std::clamp(vx, -max_speed_x_, max_speed_x_);
    vyaw = std::clamp(vyaw, -max_yaw, max_yaw);

    const bool is_stop = (vx == 0.0 && vyaw == 0.0);
    if (is_stop) {
      if (moving_) {
        publish_stop();
        moving_ = false;
      }
      return;
    }

    // 값이 바뀌었거나 하트비트 주기가 지났을 때만 발행
    const double since_pub =
      std::chrono::duration<double>(now - last_pub_).count();
    if (vx != last_vx_ || vyaw != last_vyaw_ || since_pub >= heartbeat_sec_) {
      Velocity msg;
      msg.x = static_cast<float>(vx);
      msg.y = 0.0f;  // puppy_control 은 y != 0 인 명령을 무시한다
      msg.yaw_rate = static_cast<float>(vyaw);
      velocity_pub_->publish(msg);

      last_vx_ = vx;
      last_vyaw_ = vyaw;
      last_pub_ = now;
      moving_ = true;
      stop_repeat_ = 3;

      if (debug_) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[발행] x=%+.1f cm/s  yaw=%+.2f rad/s", vx, vyaw);
      }
    }
  }

  void publish_stop()
  {
    Velocity msg;  // x=0, y=0, yaw_rate=0 → puppy_control 이 move_stop 수행
    velocity_pub_->publish(msg);
    last_vx_ = 0.0;
    last_vyaw_ = 0.0;
    last_pub_ = std::chrono::steady_clock::now();
  }

  // ── 파라미터 ──
  int port_{5005};
  double deadzone_deg_{5.0};
  double max_angle_deg_{45.0};
  double recv_timeout_sec_{1.0};
  double max_speed_x_{15.0};
  double max_yaw_rate_deg_{20.0};
  bool invert_forward_{false};
  bool invert_turn_{true};
  std::string velocity_topic_;
  double publish_rate_{20.0};
  double speed_step_{1.0};
  double yaw_step_deg_{2.0};
  double heartbeat_sec_{0.5};
  bool debug_{false};

  // ── 통신/상태 ──
  int sock_fd_{-1};
  std::atomic<bool> running_{true};
  std::atomic<bool> estop_{false};
  std::thread rx_thread_;

  std::mutex cmd_mutex_;
  float cmd_x_angle_{0.0f};
  float cmd_z_angle_{0.0f};
  bool have_cmd_{false};
  std::chrono::steady_clock::time_point last_rx_{};

  bool moving_{false};
  int stop_repeat_{0};
  double last_vx_{0.0};
  double last_vyaw_{0.0};
  std::chrono::steady_clock::time_point last_pub_{};

  rclcpp::Publisher<Velocity>::SharedPtr velocity_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr go_home_client_;
  rclcpp::TimerBase::SharedPtr stand_timer_;
  int stand_tries_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VrUdpTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
