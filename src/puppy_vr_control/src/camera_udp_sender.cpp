// camera_udp_sender.cpp
// ─────────────────────────────────────────────────────────────────────────────
// 로봇 카메라 영상(/image_raw/compressed, JPEG)을 UDP로 VR 헤드셋(Quest/Unity)에
// 전송하는 노드.
//
// 왜 청크로 쪼개는가:
//   640x480 JPEG 한 장은 보통 20~60KB. UDP 데이터그램 한계(약 65,507바이트)에는
//   들어가지만, MTU(1,500바이트)를 넘으면 IP 계층에서 단편화되고 Wi-Fi에서는
//   단편 하나만 유실돼도 프레임 전체가 사라진다. 그래서 애초에 MTU 이하 크기의
//   청크로 잘라 보내고 Unity(C#) 쪽에서 재조립한다.
//
// 패킷 구조 (모든 정수는 네트워크 바이트 순서, 총 12바이트 헤더 + JPEG 조각):
//   [0..3]  uint32 frame_id     : 프레임 번호 (재조립 그룹 구분)
//   [4..5]  uint16 chunk_index  : 이 조각의 순번 (0부터)
//   [6..7]  uint16 chunk_count  : 전체 조각 수
//   [8..11] uint32 frame_size   : JPEG 전체 크기(바이트)
//   [12.. ] JPEG 데이터 조각
//
// 수신 대상 결정 방식:
//   - client_ip 파라미터를 지정하면 그 주소로 고정 전송
//   - 지정하지 않으면(빈 문자열) Quest가 bind_port로 아무 패킷("hello" 등)을
//     보내는 순간 그 발신 주소를 기억해서 스트리밍 시작 (자동 발견).
//     client_timeout_sec 동안 hello가 다시 안 오면 전송 중단.
// ─────────────────────────────────────────────────────────────────────────────

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

using namespace std::chrono_literals;
using CompressedImage = sensor_msgs::msg::CompressedImage;

class CameraUdpSender : public rclcpp::Node
{
public:
  CameraUdpSender()
  : Node("camera_udp_sender")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/image_raw/compressed");
    client_ip_ = declare_parameter<std::string>("client_ip", "");   // ""=hello 패킷으로 자동 발견
    client_port_ = declare_parameter<int>("client_port", 5006);     // 고정 전송 시 목적지 포트
    bind_port_ = declare_parameter<int>("bind_port", 5006);         // hello 수신 포트
    max_fps_ = declare_parameter<double>("max_fps", 15.0);
    chunk_size_ = declare_parameter<int>("chunk_size", 1400);       // MTU(1500) 이하 권장
    client_timeout_sec_ = declare_parameter<double>("client_timeout_sec", 5.0);

    open_socket();

    // client_ip 를 지정한 경우: 고정 목적지 설정
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
        "자동 발견 모드: Quest가 UDP %d 포트로 패킷을 보내면 그 주소로 스트리밍 시작",
        bind_port_);
      hello_thread_ = std::thread(&CameraUdpSender::hello_loop, this);
    }

    // 카메라 토픽은 SensorDataQoS(best-effort) — usb_cam 발행 QoS와 호환
    image_sub_ = create_subscription<CompressedImage>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CameraUdpSender::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "구독: %s  (최대 %.1f fps, 청크 %d 바이트)",
      image_topic_.c_str(), max_fps_, chunk_size_);
  }

  ~CameraUdpSender() override
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

  // Quest가 보내는 hello 패킷의 발신 주소를 기억한다 (자동 발견 모드)
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
        RCLCPP_INFO(get_logger(), "클라이언트 발견: %s:%d → 스트리밍 시작",
          ip, ntohs(src.sin_port));
      }
    }
  }

  void image_callback(const CompressedImage::SharedPtr msg)
  {
    // fps 제한: max_fps 를 넘는 프레임은 버린다 (Wi-Fi 대역폭 보호)
    const auto now = std::chrono::steady_clock::now();
    const double since_last =
      std::chrono::duration<double>(now - last_send_).count();
    if (max_fps_ > 0.0 && since_last < 1.0 / max_fps_) {
      return;
    }

    sockaddr_in dest{};
    {
      std::lock_guard<std::mutex> lock(client_mutex_);
      if (!have_client_) {
        return;
      }
      // 자동 발견 모드에서는 hello가 끊기면 전송 중단
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

    const auto & jpeg = msg->data;
    const size_t total = jpeg.size();
    if (total == 0) {
      return;
    }
    const size_t chunk = static_cast<size_t>(chunk_size_);
    const size_t count = (total + chunk - 1) / chunk;
    if (count > 65535) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "프레임이 너무 큼(%zu 바이트) → 건너뜀", total);
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
      std::memcpy(packet.data() + 12, jpeg.data() + offset, len);

      ::sendto(
        sock_fd_, packet.data(), 12 + len, 0,
        reinterpret_cast<const sockaddr *>(&dest), sizeof(dest));
    }
    last_send_ = now;
  }

  // ── 파라미터 ──
  std::string image_topic_;
  std::string client_ip_;
  int client_port_{5006};
  int bind_port_{5006};
  double max_fps_{15.0};
  int chunk_size_{1400};
  double client_timeout_sec_{5.0};

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

  rclcpp::Subscription<CompressedImage>::SharedPtr image_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraUdpSender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
