// map_udp_sender.cpp
// ─────────────────────────────────────────────────────────────────────────────
// lidar_mapping_cpp 가 만든 점유격자 지도(/map, nav_msgs/OccupancyGrid)를
// PNG 이미지로 인코딩해 UDP로 노트북 대시보드(tools/vr_test_dashboard.py)나
// Quest 에 전송하는 노드. 카메라 영상(camera_udp_sender)과 완전히 동일한
// 청크 프로토콜/hello 자동발견을 쓰되, 포트만 5008 이다.
//
// 왜 별도 노드인가:
//   대시보드는 ROS 없이 UDP 만으로 로봇과 통신한다(설계 원칙). 로봇 안에서
//   ROS 토픽인 /map 을 이미지로 바꿔 UDP 로 내보내는 다리가 이 노드다.
//   대시보드 쪽 수신·표시(map_loop, 하단 지도 패널)는 이미 구현돼 있으므로,
//   이 노드만 있으면 "라이다 지도 만들어지는 화면"이 그대로 뜬다.
//
// 왜 PNG 인가 (JPEG 아님):
//   지도는 흰(빈)/검(점유)/회색(미탐사)의 경계가 뚜렷한 영역 이미지라 PNG 무손실
//   압축이 매우 잘 되고(보통 수 KB → 1청크), 경계가 뭉개지지 않는다. JPEG 는
//   블록 경계에 아티팩트가 생겨 지도에는 부적합하다. pygame.image.load 는 PNG 를
//   그대로 읽는다(대시보드 코드 변경 불필요).
//
// 패킷 구조: camera_udp_sender 와 동일 (12바이트 헤더 + PNG 조각, 네트워크 바이트 순서)
//   [0..3] frame_id  [4..5] chunk_index  [6..7] chunk_count  [8..11] frame_size
// ─────────────────────────────────────────────────────────────────────────────

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

class MapUdpSender : public rclcpp::Node
{
public:
  MapUdpSender()
  : Node("map_udp_sender")
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    client_ip_ = declare_parameter<std::string>("client_ip", "");   // ""=hello 자동 발견
    client_port_ = declare_parameter<int>("client_port", 5008);     // 고정 전송 시 목적지 포트
    bind_port_ = declare_parameter<int>("bind_port", 5008);         // hello 수신 포트
    max_hz_ = declare_parameter<double>("max_hz", 5.0);             // 전송 상한(지도는 보통 1Hz)
    chunk_size_ = declare_parameter<int>("chunk_size", 1400);       // MTU(1500) 이하 권장
    client_timeout_sec_ = declare_parameter<double>("client_timeout_sec", 5.0);
    unknown_gray_ = declare_parameter<int>("unknown_gray", 128);    // 미탐사 셀 밝기(0~255)

    open_socket();

    if (!client_ip_.empty()) {
      std::memset(&client_addr_, 0, sizeof(client_addr_));
      client_addr_.sin_family = AF_INET;
      client_addr_.sin_port = htons(static_cast<uint16_t>(client_port_));
      if (::inet_pton(AF_INET, client_ip_.c_str(), &client_addr_.sin_addr) != 1) {
        throw std::runtime_error("client_ip 형식이 잘못됨: " + client_ip_);
      }
      have_client_ = true;
      RCLCPP_INFO(get_logger(), "고정 목적지로 전송: %s:%d", client_ip_.c_str(), client_port_);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "자동 발견 모드: 대시보드가 UDP %d 포트로 hello 를 보내면 그 주소로 전송 시작",
        bind_port_);
      hello_thread_ = std::thread(&MapUdpSender::hello_loop, this);
    }

    // /map 은 lidar_mapping 이 transient_local(latched) 로 발행 → 같은 QoS 로 구독해야
    // 늦게 켜도 마지막 지도를 즉시 받는다.
    map_sub_ = create_subscription<OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&MapUdpSender::map_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "구독: %s  (최대 %.1f Hz, 청크 %d 바이트, PNG 전송 → :%d)",
      map_topic_.c_str(), max_hz_, chunk_size_, bind_port_);
  }

  ~MapUdpSender() override
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
    if (sock_fd_ < 0) {
      throw std::runtime_error("UDP 소켓 생성 실패");
    }
    const int reuse = 1;
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(bind_port_));
    if (::bind(sock_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      ::close(sock_fd_);
      sock_fd_ = -1;
      throw std::runtime_error(
        "UDP 포트 " + std::to_string(bind_port_) + " 바인드 실패");
    }
  }

  // 대시보드가 보내는 hello 패킷의 발신 주소를 기억한다 (자동 발견 모드)
  void hello_loop()
  {
    char buf[64];
    while (running_ && rclcpp::ok()) {
      pollfd pfd{sock_fd_, POLLIN, 0};
      const int rc = ::poll(&pfd, 1, 200);
      if (rc <= 0 || !(pfd.revents & POLLIN)) {
        continue;
      }
      sockaddr_in src{};
      socklen_t src_len = sizeof(src);
      const ssize_t n = ::recvfrom(
        sock_fd_, buf, sizeof(buf), MSG_DONTWAIT,
        reinterpret_cast<sockaddr *>(&src), &src_len);
      if (n < 0) {
        continue;
      }
      std::lock_guard<std::mutex> lock(client_mutex_);
      const bool is_new =
        !have_client_ || src.sin_addr.s_addr != client_addr_.sin_addr.s_addr ||
        src.sin_port != client_addr_.sin_port;
      client_addr_ = src;
      have_client_ = true;
      last_hello_ = std::chrono::steady_clock::now();
      if (is_new) {
        char ip[INET_ADDRSTRLEN] = "?";
        ::inet_ntop(AF_INET, &src.sin_addr, ip, sizeof(ip));
        RCLCPP_INFO(get_logger(), "클라이언트 발견: %s:%d → 지도 전송 시작",
          ip, ntohs(src.sin_port));
      }
    }
  }

  // OccupancyGrid → 회색조 PNG 바이트
  //   미탐사(-1)=회색, 빈공간(0)=흰색, 점유(100)=검은색.
  //   OccupancyGrid 는 row 0 이 아래쪽(y 최소)이라, 이미지가 정립하도록 세로로 뒤집는다.
  bool encode_png(const OccupancyGrid & grid, std::vector<uint8_t> & out) const
  {
    const int w = static_cast<int>(grid.info.width);
    const int h = static_cast<int>(grid.info.height);
    if (w <= 0 || h <= 0 ||
      grid.data.size() < static_cast<size_t>(w) * static_cast<size_t>(h))
    {
      return false;
    }
    const uchar unknown = static_cast<uchar>(std::clamp(unknown_gray_, 0, 255));
    cv::Mat img(h, w, CV_8UC1);
    for (int gy = 0; gy < h; ++gy) {
      uchar * row = img.ptr<uchar>(h - 1 - gy);   // 세로 뒤집기
      const int8_t * src = reinterpret_cast<const int8_t *>(grid.data.data()) +
        static_cast<size_t>(gy) * static_cast<size_t>(w);
      for (int gx = 0; gx < w; ++gx) {
        const int8_t v = src[gx];
        row[gx] = (v < 0) ? unknown
                          : static_cast<uchar>(255 - static_cast<int>(v) * 255 / 100);
      }
    }
    return cv::imencode(".png", img, out);
  }

  void map_callback(const OccupancyGrid::SharedPtr msg)
  {
    // 전송 상한 (Wi-Fi 대역폭 보호). 지도는 보통 1Hz 라 사실상 통과.
    const auto now = std::chrono::steady_clock::now();
    const double since_last =
      std::chrono::duration<double>(now - last_send_).count();
    if (max_hz_ > 0.0 && since_last < 1.0 / max_hz_) {
      return;
    }

    sockaddr_in dest{};
    {
      std::lock_guard<std::mutex> lock(client_mutex_);
      if (!have_client_) {
        return;
      }
      if (client_ip_.empty()) {
        const double hello_age =
          std::chrono::duration<double>(now - last_hello_).count();
        if (hello_age > client_timeout_sec_) {
          if (!timeout_logged_) {
            RCLCPP_WARN(get_logger(), "클라이언트 hello 끊김(%.1fs) → 전송 중단",
              client_timeout_sec_);
            timeout_logged_ = true;
          }
          return;
        }
      }
      dest = client_addr_;
      timeout_logged_ = false;
    }

    std::vector<uint8_t> png;
    if (!encode_png(*msg, png) || png.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "지도 PNG 인코딩 실패 (width=%u height=%u)", msg->info.width, msg->info.height);
      return;
    }

    const size_t total = png.size();
    const size_t chunk = static_cast<size_t>(chunk_size_);
    const size_t count = (total + chunk - 1) / chunk;
    if (count > 65535) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "지도가 너무 큼(%zu 바이트) → 건너뜀", total);
      return;
    }

    std::vector<uint8_t> packet(12 + chunk);
    const uint32_t frame_id = htonl(frame_counter_++);
    const uint16_t chunk_count = htons(static_cast<uint16_t>(count));
    const uint32_t frame_size = htonl(static_cast<uint32_t>(total));

    for (size_t i = 0; i < count; ++i) {
      const size_t offset = i * chunk;
      const size_t len = std::min(chunk, total - offset);
      const uint16_t chunk_index = htons(static_cast<uint16_t>(i));

      std::memcpy(packet.data() + 0, &frame_id, 4);
      std::memcpy(packet.data() + 4, &chunk_index, 2);
      std::memcpy(packet.data() + 6, &chunk_count, 2);
      std::memcpy(packet.data() + 8, &frame_size, 4);
      std::memcpy(packet.data() + 12, png.data() + offset, len);

      ::sendto(
        sock_fd_, packet.data(), 12 + len, 0,
        reinterpret_cast<const sockaddr *>(&dest), sizeof(dest));
    }
    last_send_ = now;
  }

  // ── 파라미터 ──
  std::string map_topic_;
  std::string client_ip_;
  int client_port_{5008};
  int bind_port_{5008};
  double max_hz_{5.0};
  int chunk_size_{1400};
  double client_timeout_sec_{5.0};
  int unknown_gray_{128};

  // ── 통신/상태 ──
  int sock_fd_{-1};
  std::atomic<bool> running_{true};
  std::thread hello_thread_;

  std::mutex client_mutex_;
  sockaddr_in client_addr_{};
  bool have_client_{false};
  bool timeout_logged_{false};
  std::chrono::steady_clock::time_point last_hello_{};

  uint32_t frame_counter_{0};
  std::chrono::steady_clock::time_point last_send_{};

  rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapUdpSender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
