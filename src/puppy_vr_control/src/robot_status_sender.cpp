// robot_status_sender.cpp
// ─────────────────────────────────────────────────────────────────────────────
// 로봇 상태(배터리 전압, Wi-Fi 신호세기, ROS 링크 상태)를 UDP 로 VR 헤드셋에
// 1초마다 전송하는 노드.
//
// 프로토콜 (조이스틱/카메라와 같은 자동 발견 방식):
//   1) Quest 가 UDP 5007 포트로 아무 패킷("hello")을 주기적으로 전송
//   2) 로봇이 그 발신 주소로 상태 문자열을 1Hz 푸시:
//        "BAT:7400;BAT_AGE:0.4;RSSI:-52;UP:123"
//      - BAT     : 배터리 전압(mV). 아직 수신 전이면 -1
//      - BAT_AGE : 마지막 배터리 토픽 수신 후 경과(초) — 로봇 내부 ROS 링크 건강도
//                  (3초 이상이면 ros_robot_controller 가 죽었거나 보드 통신 이상)
//      - RSSI    : Wi-Fi 신호세기(dBm, /proc/net/wireless). 읽기 실패 시 0
//      - UP      : 노드 가동 시간(초)
//   3) Quest 쪽 "로봇 연결됨" 판정: 이 패킷이 3초 이상 안 오면 연결 끊김으로 표시
//
// Unity 파싱 예시는 공부자료/05_VR_Unity_UDP_통신.md 참고.
// ─────────────────────────────────────────────────────────────────────────────

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"

using namespace std::chrono_literals;

class RobotStatusSender : public rclcpp::Node
{
public:
  RobotStatusSender()
  : Node("robot_status_sender"), start_time_(std::chrono::steady_clock::now())
  {
    bind_port_ = declare_parameter<int>("bind_port", 5007);
    client_ip_ = declare_parameter<std::string>("client_ip", "");   // ""=자동 발견
    client_port_ = declare_parameter<int>("client_port", 5007);
    send_period_ = declare_parameter<double>("send_period", 1.0);
    client_timeout_sec_ = declare_parameter<double>("client_timeout_sec", 5.0);
    wireless_if_ = declare_parameter<std::string>("wireless_if", "wlan0");
    battery_topic_ = declare_parameter<std::string>(
      "battery_topic", "/ros_robot_controller/battery");

    open_socket();
    if (!client_ip_.empty()) {
      std::memset(&client_addr_, 0, sizeof(client_addr_));
      client_addr_.sin_family = AF_INET;
      client_addr_.sin_port = htons(static_cast<uint16_t>(client_port_));
      ::inet_pton(AF_INET, client_ip_.c_str(), &client_addr_.sin_addr);
      have_client_ = true;
    } else {
      hello_thread_ = std::thread(&RobotStatusSender::hello_loop, this);
    }

    battery_sub_ = create_subscription<std_msgs::msg::UInt16>(
      battery_topic_, 1, [this](std_msgs::msg::UInt16::SharedPtr msg) {
        battery_mv_ = msg->data;
        last_battery_ = std::chrono::steady_clock::now();
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(send_period_),
      std::bind(&RobotStatusSender::send_status, this));

    RCLCPP_INFO(
      get_logger(), "상태 전송 대기: UDP %d (배터리=%s, Wi-Fi=%s)",
      bind_port_, battery_topic_.c_str(), wireless_if_.c_str());
  }

  ~RobotStatusSender() override
  {
    running_ = false;
    if (hello_thread_.joinable()) {
      hello_thread_.join();
    }
    if (sock_fd_ >= 0) {
      ::close(sock_fd_);
    }
  }

private:
  void open_socket()
  {
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    const int reuse = 1;
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(bind_port_));
    if (::bind(sock_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      throw std::runtime_error("UDP 포트 바인드 실패: " + std::to_string(bind_port_));
    }
  }

  void hello_loop()
  {
    char buf[64];
    while (running_ && rclcpp::ok()) {
      pollfd pfd{sock_fd_, POLLIN, 0};
      if (::poll(&pfd, 1, 200) <= 0 || !(pfd.revents & POLLIN)) {
        continue;
      }
      sockaddr_in src{};
      socklen_t len = sizeof(src);
      if (::recvfrom(sock_fd_, buf, sizeof(buf), MSG_DONTWAIT,
        reinterpret_cast<sockaddr *>(&src), &len) < 0)
      {
        continue;
      }
      std::lock_guard<std::mutex> lock(client_mutex_);
      if (!have_client_ || src.sin_addr.s_addr != client_addr_.sin_addr.s_addr) {
        char ip[INET_ADDRSTRLEN] = "?";
        ::inet_ntop(AF_INET, &src.sin_addr, ip, sizeof(ip));
        RCLCPP_INFO(get_logger(), "상태 수신 클라이언트: %s:%d", ip, ntohs(src.sin_port));
      }
      client_addr_ = src;
      have_client_ = true;
      last_hello_ = std::chrono::steady_clock::now();
    }
  }

  // /proc/net/wireless 에서 신호세기(dBm) 파싱 (리눅스 전용)
  int read_rssi() const
  {
    std::ifstream f("/proc/net/wireless");
    std::string line;
    while (std::getline(f, line)) {
      if (line.find(wireless_if_ + ":") == std::string::npos) {
        continue;
      }
      std::istringstream iss(line);
      std::string iface, status, quality, level;
      iss >> iface >> status >> quality >> level;   // level = "-52." 형태
      try {
        return static_cast<int>(std::stod(level));
      } catch (const std::exception &) {
        return 0;
      }
    }
    return 0;
  }

  void send_status()
  {
    sockaddr_in dest{};
    {
      std::lock_guard<std::mutex> lock(client_mutex_);
      if (!have_client_) {
        return;
      }
      if (client_ip_.empty()) {
        const double age = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_hello_).count();
        if (age > client_timeout_sec_) {
          return;  // Quest 쪽 hello 끊김 → 전송 중단
        }
      }
      dest = client_addr_;
    }

    const auto now = std::chrono::steady_clock::now();
    const double bat_age = battery_mv_ < 0 ? -1.0 :
      std::chrono::duration<double>(now - last_battery_).count();
    const int uptime = static_cast<int>(
      std::chrono::duration<double>(now - start_time_).count());

    char msg[128];
    std::snprintf(
      msg, sizeof(msg), "BAT:%d;BAT_AGE:%.1f;RSSI:%d;UP:%d",
      battery_mv_.load(), bat_age, read_rssi(), uptime);
    ::sendto(
      sock_fd_, msg, std::strlen(msg), 0,
      reinterpret_cast<const sockaddr *>(&dest), sizeof(dest));

    // 로봇 내부 링크 경고 (보드/드라이버 죽음 감지)
    if (bat_age > 3.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "배터리 토픽 %.1fs 무소식 — ros_robot_controller 상태 확인 필요", bat_age);
    }
  }

  // ── 파라미터/상태 ──
  int bind_port_{5007};
  std::string client_ip_;
  int client_port_{5007};
  double send_period_{1.0};
  double client_timeout_sec_{5.0};
  std::string wireless_if_;
  std::string battery_topic_;

  int sock_fd_{-1};
  std::atomic<bool> running_{true};
  std::thread hello_thread_;
  std::mutex client_mutex_;
  sockaddr_in client_addr_{};
  bool have_client_{false};
  std::chrono::steady_clock::time_point last_hello_{};

  std::atomic<int> battery_mv_{-1};
  std::chrono::steady_clock::time_point last_battery_{};
  const std::chrono::steady_clock::time_point start_time_;

  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr battery_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotStatusSender>());
  rclcpp::shutdown();
  return 0;
}
