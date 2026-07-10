// board.cpp — Board 클래스 구현 (시리얼 I/O + 수신 스레드)
#include "ros_robot_controller_cpp/board.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <stdexcept>

namespace ros_robot_controller_cpp
{

std::optional<std::vector<uint8_t>> LatestQueue::get_wait(double timeout_sec)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (!cv_.wait_for(
      lock, std::chrono::duration<double>(timeout_sec),
      [this] {return has_data_;}))
  {
    return std::nullopt;
  }
  has_data_ = false;
  return data_;
}

Board::Board(const std::string & device, int baudrate)
{
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    throw std::runtime_error("시리얼 포트 열기 실패: " + device);
  }

  termios tio{};
  if (::tcgetattr(fd_, &tio) != 0) {
    ::close(fd_);
    throw std::runtime_error("tcgetattr 실패");
  }
  ::cfmakeraw(&tio);
  // 보드는 1,000,000bps (파이썬 기본값과 동일). 리눅스는 B1000000 상수 지원
  speed_t speed = B1000000;
  if (baudrate == 115200) {speed = B115200;}
  ::cfsetispeed(&tio, speed);
  ::cfsetospeed(&tio, speed);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cc[VMIN] = 0;    // 논블로킹에 가까운 read:
  tio.c_cc[VTIME] = 1;   // 0.1초 안에 온 만큼만 반환
  ::tcsetattr(fd_, TCSANOW, &tio);

  // 파이썬: port.rts = False, port.dtr = False (보드 리셋 방지)
  int modem_bits = TIOCM_RTS | TIOCM_DTR;
  ::ioctl(fd_, TIOCMBIC, &modem_bits);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 파이썬 time.sleep(0.5)
  recv_thread_ = std::thread(&Board::recv_task, this);
}

Board::~Board()
{
  running_ = false;
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

void Board::write_frame(const std::vector<uint8_t> & frame)
{
  std::lock_guard<std::mutex> lock(write_mutex_);
  const uint8_t * p = frame.data();
  size_t remaining = frame.size();
  while (remaining > 0) {
    const ssize_t n = ::write(fd_, p, remaining);
    if (n <= 0) {
      return;  // 쓰기 실패 — 파이썬도 예외 없이 진행되는 케이스
    }
    p += n;
    remaining -= static_cast<size_t>(n);
  }
}

void Board::recv_task()
{
  FrameParser parser(
    [this](PacketFunction func, const std::vector<uint8_t> & data) {
      on_frame(func, data);
    });

  uint8_t buf[256];
  while (running_) {
    if (!enable_recv_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    const ssize_t n = ::read(fd_, buf, sizeof(buf));  // VTIME=0.1s
    for (ssize_t i = 0; i < n; ++i) {
      parser.feed(buf[i]);
    }
  }
}

void Board::on_frame(PacketFunction func, const std::vector<uint8_t> & data)
{
  switch (func) {
    case PacketFunction::SYS: sys_queue_.put_nowait(data); break;
    case PacketFunction::KEY: key_queue_.put_nowait(data); break;
    case PacketFunction::IMU: imu_queue_.put_nowait(data); break;
    case PacketFunction::GAMEPAD: gamepad_queue_.put_nowait(data); break;
    case PacketFunction::SBUS: sbus_queue_.put_nowait(data); break;
    case PacketFunction::BUS_SERVO: bus_servo_queue_.put_nowait(data); break;
    case PacketFunction::PWM_SERVO: pwm_servo_queue_.put_nowait(data); break;
    default: break;
  }
}

// ── 출력 명령 ──

void Board::set_led(double on_time, double off_time, int repeat, int led_id)
{
  write_frame(frame_set_led(on_time, off_time, repeat, led_id));
}

void Board::set_buzzer(int freq, double on_time, double off_time, int repeat)
{
  write_frame(frame_set_buzzer(freq, on_time, off_time, repeat));
}

void Board::set_motor_speed(const std::vector<std::pair<int, double>> & speeds)
{
  write_frame(frame_set_motor_speed(speeds));
}

void Board::set_oled_text(int line, const std::string & text)
{
  write_frame(frame_set_oled_text(line, text));
}

void Board::set_rgb(const std::vector<std::array<int, 4>> & pixels)
{
  write_frame(frame_set_rgb(pixels));
}

void Board::pwm_servo_set_position(
  double duration_s, const std::vector<std::pair<int, int>> & positions)
{
  write_frame(frame_pwm_servo_set_position(duration_s, positions));
}

void Board::pwm_servo_set_offset(int servo_id, int offset)
{
  write_frame(frame_pwm_servo_set_offset(servo_id, offset));
}

void Board::bus_servo_set_position(
  double duration_s, const std::vector<std::pair<int, int>> & positions)
{
  write_frame(frame_bus_servo_set_position(duration_s, positions));
}

// 파이썬은 bus servo 설정 명령 뒤에 20ms 대기 — 동일하게 유지
static void settle() {std::this_thread::sleep_for(std::chrono::milliseconds(20));}

void Board::bus_servo_enable_torque(int servo_id, bool enable)
{
  write_frame(frame_bus_servo_enable_torque(servo_id, enable));
  settle();
}

void Board::bus_servo_set_id(int id_now, int id_new)
{
  write_frame(frame_bus_servo_set_id(id_now, id_new));
  settle();
}

void Board::bus_servo_set_offset(int servo_id, int offset)
{
  write_frame(frame_bus_servo_set_offset(servo_id, offset));
  settle();
}

void Board::bus_servo_save_offset(int servo_id)
{
  write_frame(frame_bus_servo_save_offset(servo_id));
  settle();
}

void Board::bus_servo_set_angle_limit(int servo_id, uint16_t low, uint16_t high)
{
  write_frame(frame_bus_servo_set_angle_limit(servo_id, low, high));
  settle();
}

void Board::bus_servo_set_vin_limit(int servo_id, uint16_t low, uint16_t high)
{
  write_frame(frame_bus_servo_set_vin_limit(servo_id, low, high));
  settle();
}

void Board::bus_servo_set_temp_limit(int servo_id, int limit)
{
  write_frame(frame_bus_servo_set_temp_limit(servo_id, limit));
  settle();
}

void Board::bus_servo_stop(const std::vector<int> & servo_ids)
{
  write_frame(frame_bus_servo_stop(servo_ids));
}

// ── 상태 읽기 ──

std::optional<uint16_t> Board::get_battery()
{
  auto d = sys_queue_.get_nowait();
  return d ? decode_battery(*d) : std::nullopt;
}

std::optional<std::pair<uint8_t, uint8_t>> Board::get_button()
{
  auto d = key_queue_.get_nowait();
  return d ? decode_button(*d) : std::nullopt;
}

std::optional<std::array<float, 6>> Board::get_imu()
{
  auto d = imu_queue_.get_nowait();
  return d ? decode_imu(*d) : std::nullopt;
}

std::optional<GamepadData> Board::get_gamepad()
{
  auto d = gamepad_queue_.get_nowait();
  return d ? decode_gamepad(*d) : std::nullopt;
}

std::optional<std::vector<float>> Board::get_sbus()
{
  auto d = sbus_queue_.get_nowait();
  return d ? decode_sbus(*d) : std::nullopt;
}

// ── 요청/응답형 서보 읽기 ──
// 응답 형식(파이썬 unpack 문자열):
//   pwm : [id, cmd, value]        value = uint16(위치) 또는 int8(오프셋)
//   bus : [id, cmd, success, ...] success==0 일 때만 유효

static constexpr double READ_TIMEOUT_SEC = 1.0;

std::optional<int> Board::pwm_servo_read_position(int servo_id)
{
  std::lock_guard<std::mutex> lock(servo_read_mutex_);
  write_frame(frame_read_request(PacketFunction::PWM_SERVO, 0x05, servo_id));
  auto d = pwm_servo_queue_.get_wait(READ_TIMEOUT_SEC);
  if (!d || d->size() < 4) {return std::nullopt;}   // '<BBH'
  return static_cast<int>(get_u16(&(*d)[2]));
}

std::optional<int> Board::pwm_servo_read_offset(int servo_id)
{
  std::lock_guard<std::mutex> lock(servo_read_mutex_);
  write_frame(frame_read_request(PacketFunction::PWM_SERVO, 0x09, servo_id));
  auto d = pwm_servo_queue_.get_wait(READ_TIMEOUT_SEC);
  if (!d || d->size() < 3) {return std::nullopt;}   // '<BBb'
  return static_cast<int>(static_cast<int8_t>((*d)[2]));
}

std::optional<int> Board::bus_servo_read_id(int servo_id)
{
  std::lock_guard<std::mutex> lock(servo_read_mutex_);
  write_frame(frame_read_request(PacketFunction::BUS_SERVO, 0x12, servo_id));
  auto d = bus_servo_queue_.get_wait(READ_TIMEOUT_SEC);
  if (!d || d->size() < 4 || static_cast<int8_t>((*d)[2]) != 0) {return std::nullopt;}
  return static_cast<int>((*d)[3]);  // '<BBbB'
}

std::optional<int> Board::bus_servo_read_offset(int servo_id)
{
  std::lock_guard<std::mutex> lock(servo_read_mutex_);
  write_frame(frame_read_request(PacketFunction::BUS_SERVO, 0x22, servo_id));
  auto d = bus_servo_queue_.get_wait(READ_TIMEOUT_SEC);
  if (!d || d->size() < 4 || static_cast<int8_t>((*d)[2]) != 0) {return std::nullopt;}
  return static_cast<int>(static_cast<int8_t>((*d)[3]));  // '<BBbb'
}

std::optional<int> Board::bus_servo_read_position(int servo_id)
{
  std::lock_guard<std::mutex> lock(servo_read_mutex_);
  write_frame(frame_read_request(PacketFunction::BUS_SERVO, 0x05, servo_id));
  auto d = bus_servo_queue_.get_wait(READ_TIMEOUT_SEC);
  if (!d || d->size() < 5 || static_cast<int8_t>((*d)[2]) != 0) {return std::nullopt;}
  return static_cast<int>(get_i16(&(*d)[3]));  // '<BBbh'
}

std::optional<int> Board::bus_servo_read_vin(int servo_id)
{
  std::lock_guard<std::mutex> lock(servo_read_mutex_);
  write_frame(frame_read_request(PacketFunction::BUS_SERVO, 0x07, servo_id));
  auto d = bus_servo_queue_.get_wait(READ_TIMEOUT_SEC);
  if (!d || d->size() < 5 || static_cast<int8_t>((*d)[2]) != 0) {return std::nullopt;}
  return static_cast<int>(get_u16(&(*d)[3]));  // '<BBbH'
}

std::optional<int> Board::bus_servo_read_temp(int servo_id)
{
  std::lock_guard<std::mutex> lock(servo_read_mutex_);
  write_frame(frame_read_request(PacketFunction::BUS_SERVO, 0x09, servo_id));
  auto d = bus_servo_queue_.get_wait(READ_TIMEOUT_SEC);
  if (!d || d->size() < 4 || static_cast<int8_t>((*d)[2]) != 0) {return std::nullopt;}
  return static_cast<int>((*d)[3]);  // '<BBbB'
}

std::optional<int> Board::bus_servo_read_temp_limit(int servo_id)
{
  std::lock_guard<std::mutex> lock(servo_read_mutex_);
  write_frame(frame_read_request(PacketFunction::BUS_SERVO, 0x3A, servo_id));
  auto d = bus_servo_queue_.get_wait(READ_TIMEOUT_SEC);
  if (!d || d->size() < 4 || static_cast<int8_t>((*d)[2]) != 0) {return std::nullopt;}
  return static_cast<int>((*d)[3]);  // '<BBbB'
}

std::optional<std::pair<int, int>> Board::bus_servo_read_angle_limit(int servo_id)
{
  std::lock_guard<std::mutex> lock(servo_read_mutex_);
  write_frame(frame_read_request(PacketFunction::BUS_SERVO, 0x32, servo_id));
  auto d = bus_servo_queue_.get_wait(READ_TIMEOUT_SEC);
  if (!d || d->size() < 7 || static_cast<int8_t>((*d)[2]) != 0) {return std::nullopt;}
  return std::make_pair(
    static_cast<int>(get_u16(&(*d)[3])), static_cast<int>(get_u16(&(*d)[5])));  // '<BBb2H'
}

std::optional<std::pair<int, int>> Board::bus_servo_read_vin_limit(int servo_id)
{
  std::lock_guard<std::mutex> lock(servo_read_mutex_);
  write_frame(frame_read_request(PacketFunction::BUS_SERVO, 0x36, servo_id));
  auto d = bus_servo_queue_.get_wait(READ_TIMEOUT_SEC);
  if (!d || d->size() < 7 || static_cast<int8_t>((*d)[2]) != 0) {return std::nullopt;}
  return std::make_pair(
    static_cast<int>(get_u16(&(*d)[3])), static_cast<int>(get_u16(&(*d)[5])));
}

std::optional<int> Board::bus_servo_read_torque_state(int servo_id)
{
  std::lock_guard<std::mutex> lock(servo_read_mutex_);
  write_frame(frame_read_request(PacketFunction::BUS_SERVO, 0x0D, servo_id));
  auto d = bus_servo_queue_.get_wait(READ_TIMEOUT_SEC);
  if (!d || d->size() < 4 || static_cast<int8_t>((*d)[2]) != 0) {return std::nullopt;}
  return static_cast<int>(static_cast<int8_t>((*d)[3]));  // '<BBbb'
}

}  // namespace ros_robot_controller_cpp
