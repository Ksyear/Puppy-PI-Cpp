// board.hpp
// ─────────────────────────────────────────────────────────────────────────────
// STM32 확장보드 통신 클래스 (ros_robot_controller_sdk.py 의 Board 이식).
// /dev/rrc @ 1,000,000bps, RTS/DTR off. 수신 스레드가 프레임을 파싱해
// 기능별 1칸짜리 큐에 넣는다 (가득 차면 버림 — 파이썬 put_nowait 와 동일).
// ─────────────────────────────────────────────────────────────────────────────
#ifndef ROS_ROBOT_CONTROLLER_CPP__BOARD_HPP_
#define ROS_ROBOT_CONTROLLER_CPP__BOARD_HPP_

#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ros_robot_controller_cpp/board_protocol.hpp"

namespace ros_robot_controller_cpp
{

// 파이썬 queue.Queue(maxsize=1) 대응: 가득 차면 새 데이터를 버린다
class LatestQueue
{
public:
  void put_nowait(const std::vector<uint8_t> & data)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_data_) {
      data_ = data;
      has_data_ = true;
      cv_.notify_one();
    }
  }

  std::optional<std::vector<uint8_t>> get_nowait()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_data_) {return std::nullopt;}
    has_data_ = false;
    return data_;
  }

  // 서보 읽기 응답 대기용. 파이썬은 무한 대기지만 여기서는 타임아웃을 둬서
  // 보드 무응답 시 서비스가 영원히 멈추는 것을 막는다 (의도적 개선)
  std::optional<std::vector<uint8_t>> get_wait(double timeout_sec);

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<uint8_t> data_;
  bool has_data_{false};
};

class Board
{
public:
  explicit Board(const std::string & device = "/dev/rrc", int baudrate = 1000000);
  ~Board();

  void enable_reception(bool enable = true) {enable_recv_ = enable;}

  // ── 출력 명령 (프레임 바이트는 board_protocol.hpp 와 파이썬이 동일함을 검증) ──
  void set_led(double on_time, double off_time, int repeat = 1, int led_id = 1);
  void set_buzzer(int freq, double on_time, double off_time, int repeat = 1);
  void set_motor_speed(const std::vector<std::pair<int, double>> & speeds);
  void set_oled_text(int line, const std::string & text);
  void set_rgb(const std::vector<std::array<int, 4>> & pixels);
  void pwm_servo_set_position(double duration_s, const std::vector<std::pair<int, int>> & positions);
  void pwm_servo_set_offset(int servo_id, int offset);
  void bus_servo_set_position(double duration_s, const std::vector<std::pair<int, int>> & positions);
  void bus_servo_enable_torque(int servo_id, bool enable);
  void bus_servo_set_id(int id_now, int id_new);
  void bus_servo_set_offset(int servo_id, int offset);
  void bus_servo_save_offset(int servo_id);
  void bus_servo_set_angle_limit(int servo_id, uint16_t low, uint16_t high);
  void bus_servo_set_vin_limit(int servo_id, uint16_t low, uint16_t high);
  void bus_servo_set_temp_limit(int servo_id, int limit);
  void bus_servo_stop(const std::vector<int> & servo_ids);

  // ── 상태 읽기 (수신 큐에서 꺼내 해석; 없으면 nullopt) ──
  std::optional<uint16_t> get_battery();
  std::optional<std::pair<uint8_t, uint8_t>> get_button();
  std::optional<std::array<float, 6>> get_imu();
  std::optional<GamepadData> get_gamepad();
  std::optional<std::vector<float>> get_sbus();

  // ── 요청/응답형 서보 읽기 (파이썬 *_read_and_unpack 대응) ──
  std::optional<int> pwm_servo_read_position(int servo_id);
  std::optional<int> pwm_servo_read_offset(int servo_id);
  std::optional<int> bus_servo_read_id(int servo_id = 254);
  std::optional<int> bus_servo_read_offset(int servo_id);
  std::optional<int> bus_servo_read_position(int servo_id);
  std::optional<int> bus_servo_read_vin(int servo_id);
  std::optional<int> bus_servo_read_temp(int servo_id);
  std::optional<int> bus_servo_read_temp_limit(int servo_id);
  std::optional<std::pair<int, int>> bus_servo_read_angle_limit(int servo_id);
  std::optional<std::pair<int, int>> bus_servo_read_vin_limit(int servo_id);
  std::optional<int> bus_servo_read_torque_state(int servo_id);

private:
  void write_frame(const std::vector<uint8_t> & frame);
  void recv_task();
  void on_frame(PacketFunction func, const std::vector<uint8_t> & data);

  int fd_{-1};
  std::atomic<bool> enable_recv_{false};
  std::atomic<bool> running_{true};
  std::thread recv_thread_;
  std::mutex write_mutex_;
  std::mutex servo_read_mutex_;  // 파이썬 servo_read_lock

  LatestQueue sys_queue_;
  LatestQueue key_queue_;
  LatestQueue imu_queue_;
  LatestQueue gamepad_queue_;
  LatestQueue sbus_queue_;
  LatestQueue bus_servo_queue_;
  LatestQueue pwm_servo_queue_;
};

}  // namespace ros_robot_controller_cpp

#endif  // ROS_ROBOT_CONTROLLER_CPP__BOARD_HPP_
