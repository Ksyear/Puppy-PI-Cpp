// board_protocol.hpp
// ─────────────────────────────────────────────────────────────────────────────
// STM32 확장보드 시리얼 프로토콜의 순수 로직 (ROS/시리얼 의존 없음 → 단독 테스트 가능).
// ros_robot_controller_sdk.py 의 프레임 생성/파싱을 바이트 단위로 동일하게 이식.
//
// 프레임: 0xAA 0x55 | Function | Length | Data... | CRC8
//   (CRC8 은 Function 부터 Data 끝까지에 대해 계산)
// 모든 다중바이트 정수/실수는 리틀엔디언 (파이썬 struct '<' 와 동일).
// ─────────────────────────────────────────────────────────────────────────────
#ifndef ROS_ROBOT_CONTROLLER_CPP__BOARD_PROTOCOL_HPP_
#define ROS_ROBOT_CONTROLLER_CPP__BOARD_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <cstring>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace ros_robot_controller_cpp
{

enum class PacketFunction : uint8_t
{
  SYS = 0,
  LED = 1,
  BUZZER = 2,
  MOTOR = 3,
  PWM_SERVO = 4,
  BUS_SERVO = 5,
  KEY = 6,
  IMU = 7,
  GAMEPAD = 8,
  SBUS = 9,
  OLED = 10,
  RGB = 11,
  NONE = 12,
};

// 파이썬 crc8_table 과 동일 (Dallas/Maxim CRC8)
inline const std::array<uint8_t, 256> & crc8_table()
{
  static const std::array<uint8_t, 256> table = {{
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53,
  }};
  return table;
}

inline uint8_t checksum_crc8(const uint8_t * data, size_t len)
{
  uint8_t check = 0;
  for (size_t i = 0; i < len; ++i) {
    check = crc8_table()[check ^ data[i]];
  }
  return check;
}

// ── 리틀엔디언 직렬화 (파이썬 struct '<' 대응) ──
inline void put_u8(std::vector<uint8_t> & v, uint8_t x) {v.push_back(x);}
inline void put_i8(std::vector<uint8_t> & v, int8_t x) {v.push_back(static_cast<uint8_t>(x));}
inline void put_u16(std::vector<uint8_t> & v, uint16_t x)
{
  v.push_back(x & 0xFF);
  v.push_back((x >> 8) & 0xFF);
}
inline void put_f32(std::vector<uint8_t> & v, float x)
{
  uint8_t b[4];
  std::memcpy(b, &x, 4);  // ARM/x86 은 리틀엔디언 IEEE754 → '<f' 와 동일
  v.insert(v.end(), b, b + 4);
}
inline uint16_t get_u16(const uint8_t * p) {return p[0] | (p[1] << 8);}
inline int16_t get_i16(const uint8_t * p) {return static_cast<int16_t>(get_u16(p));}
inline float get_f32(const uint8_t * p)
{
  float f;
  std::memcpy(&f, p, 4);
  return f;
}

// 파이썬 buf_write(): 완성 프레임 생성
inline std::vector<uint8_t> build_frame(PacketFunction func, const std::vector<uint8_t> & data)
{
  std::vector<uint8_t> buf = {0xAA, 0x55, static_cast<uint8_t>(func),
    static_cast<uint8_t>(data.size())};
  buf.insert(buf.end(), data.begin(), data.end());
  buf.push_back(checksum_crc8(buf.data() + 2, buf.size() - 2));
  return buf;
}

// ── 명령별 프레임 생성 (파이썬 동명 메서드와 바이트 동일) ──

inline std::vector<uint8_t> frame_set_led(
  double on_time, double off_time, int repeat = 1, int led_id = 1)
{
  std::vector<uint8_t> d;
  put_u8(d, static_cast<uint8_t>(led_id));
  put_u16(d, static_cast<uint16_t>(static_cast<int>(on_time * 1000)));
  put_u16(d, static_cast<uint16_t>(static_cast<int>(off_time * 1000)));
  put_u16(d, static_cast<uint16_t>(repeat));
  return build_frame(PacketFunction::LED, d);
}

inline std::vector<uint8_t> frame_set_buzzer(
  int freq, double on_time, double off_time, int repeat = 1)
{
  std::vector<uint8_t> d;
  put_u16(d, static_cast<uint16_t>(freq));
  put_u16(d, static_cast<uint16_t>(static_cast<int>(on_time * 1000)));
  put_u16(d, static_cast<uint16_t>(static_cast<int>(off_time * 1000)));
  put_u16(d, static_cast<uint16_t>(repeat));
  return build_frame(PacketFunction::BUZZER, d);
}

// speeds: (모터 id(1부터), 초당 회전수) — 보드는 id-1 을 받는다
inline std::vector<uint8_t> frame_set_motor_speed(
  const std::vector<std::pair<int, double>> & speeds)
{
  std::vector<uint8_t> d = {0x01, static_cast<uint8_t>(speeds.size())};
  for (const auto & s : speeds) {
    put_u8(d, static_cast<uint8_t>(s.first - 1));
    put_f32(d, static_cast<float>(s.second));
  }
  return build_frame(PacketFunction::MOTOR, d);
}

inline std::vector<uint8_t> frame_set_oled_text(int line, const std::string & text)
{
  std::vector<uint8_t> d = {static_cast<uint8_t>(line), static_cast<uint8_t>(text.size())};
  d.insert(d.end(), text.begin(), text.end());
  return build_frame(PacketFunction::OLED, d);
}

// pixels: (index(1부터), r, g, b)
inline std::vector<uint8_t> frame_set_rgb(
  const std::vector<std::array<int, 4>> & pixels)
{
  std::vector<uint8_t> d = {0x01, static_cast<uint8_t>(pixels.size())};
  for (const auto & p : pixels) {
    put_u8(d, static_cast<uint8_t>(p[0] - 1));
    put_u8(d, static_cast<uint8_t>(p[1]));
    put_u8(d, static_cast<uint8_t>(p[2]));
    put_u8(d, static_cast<uint8_t>(p[3]));
  }
  return build_frame(PacketFunction::RGB, d);
}

// positions: (서보 id, 펄스)
inline std::vector<uint8_t> frame_pwm_servo_set_position(
  double duration_s, const std::vector<std::pair<int, int>> & positions)
{
  const int duration = static_cast<int>(duration_s * 1000);
  std::vector<uint8_t> d = {0x01,
    static_cast<uint8_t>(duration & 0xFF),
    static_cast<uint8_t>((duration >> 8) & 0xFF),
    static_cast<uint8_t>(positions.size())};
  for (const auto & p : positions) {
    put_u8(d, static_cast<uint8_t>(p.first));
    put_u16(d, static_cast<uint16_t>(p.second));
  }
  return build_frame(PacketFunction::PWM_SERVO, d);
}

inline std::vector<uint8_t> frame_pwm_servo_set_offset(int servo_id, int offset)
{
  std::vector<uint8_t> d;
  put_u8(d, 0x07);
  put_u8(d, static_cast<uint8_t>(servo_id));
  put_i8(d, static_cast<int8_t>(offset));
  return build_frame(PacketFunction::PWM_SERVO, d);
}

// 읽기 요청 (pwm: 위치 0x05 / 오프셋 0x09, bus: 각 read 명령)
inline std::vector<uint8_t> frame_read_request(PacketFunction func, uint8_t cmd, uint8_t servo_id)
{
  return build_frame(func, {cmd, servo_id});
}

inline std::vector<uint8_t> frame_bus_servo_set_position(
  double duration_s, const std::vector<std::pair<int, int>> & positions)
{
  const int duration = static_cast<int>(duration_s * 1000);
  std::vector<uint8_t> d = {0x01,
    static_cast<uint8_t>(duration & 0xFF),
    static_cast<uint8_t>((duration >> 8) & 0xFF),
    static_cast<uint8_t>(positions.size())};
  for (const auto & p : positions) {
    put_u8(d, static_cast<uint8_t>(p.first));
    put_u16(d, static_cast<uint16_t>(p.second));
  }
  return build_frame(PacketFunction::BUS_SERVO, d);
}

inline std::vector<uint8_t> frame_bus_servo_enable_torque(int servo_id, bool enable)
{
  return build_frame(
    PacketFunction::BUS_SERVO,
    {enable ? uint8_t{0x0B} : uint8_t{0x0C}, static_cast<uint8_t>(servo_id)});
}

inline std::vector<uint8_t> frame_bus_servo_set_id(int id_now, int id_new)
{
  return build_frame(
    PacketFunction::BUS_SERVO,
    {0x10, static_cast<uint8_t>(id_now), static_cast<uint8_t>(id_new)});
}

inline std::vector<uint8_t> frame_bus_servo_set_offset(int servo_id, int offset)
{
  std::vector<uint8_t> d;
  put_u8(d, 0x20);
  put_u8(d, static_cast<uint8_t>(servo_id));
  put_i8(d, static_cast<int8_t>(offset));
  return build_frame(PacketFunction::BUS_SERVO, d);
}

inline std::vector<uint8_t> frame_bus_servo_save_offset(int servo_id)
{
  return build_frame(PacketFunction::BUS_SERVO, {0x24, static_cast<uint8_t>(servo_id)});
}

inline std::vector<uint8_t> frame_bus_servo_set_angle_limit(
  int servo_id, uint16_t low, uint16_t high)
{
  std::vector<uint8_t> d;
  put_u8(d, 0x30);
  put_u8(d, static_cast<uint8_t>(servo_id));
  put_u16(d, low);
  put_u16(d, high);
  return build_frame(PacketFunction::BUS_SERVO, d);
}

inline std::vector<uint8_t> frame_bus_servo_set_vin_limit(
  int servo_id, uint16_t low, uint16_t high)
{
  std::vector<uint8_t> d;
  put_u8(d, 0x34);
  put_u8(d, static_cast<uint8_t>(servo_id));
  put_u16(d, low);
  put_u16(d, high);
  return build_frame(PacketFunction::BUS_SERVO, d);
}

inline std::vector<uint8_t> frame_bus_servo_set_temp_limit(int servo_id, int limit)
{
  std::vector<uint8_t> d;
  put_u8(d, 0x38);
  put_u8(d, static_cast<uint8_t>(servo_id));
  put_i8(d, static_cast<int8_t>(limit));
  return build_frame(PacketFunction::BUS_SERVO, d);
}

inline std::vector<uint8_t> frame_bus_servo_stop(const std::vector<int> & servo_ids)
{
  std::vector<uint8_t> d = {0x03, static_cast<uint8_t>(servo_ids.size())};
  for (int id : servo_ids) {
    put_u8(d, static_cast<uint8_t>(id));
  }
  return build_frame(PacketFunction::BUS_SERVO, d);
}

// ── 수신 프레임 파서 (파이썬 recv_task 상태머신과 동일) ──
class FrameParser
{
public:
  using Callback = std::function<void (PacketFunction, const std::vector<uint8_t> &)>;

  explicit FrameParser(Callback cb)
  : cb_(std::move(cb)) {}

  void feed(uint8_t dat)
  {
    switch (state_) {
      case State::START1:
        if (dat == 0xAA) {state_ = State::START2;}
        break;
      case State::START2:
        state_ = (dat == 0x55) ? State::FUNCTION : State::START1;
        break;
      case State::FUNCTION:
        if (dat < static_cast<uint8_t>(PacketFunction::NONE)) {
          func_ = static_cast<PacketFunction>(dat);
          data_.clear();
          state_ = State::LENGTH;
        } else {
          state_ = State::START1;
        }
        break;
      case State::LENGTH:
        length_ = dat;
        recv_count_ = 0;
        state_ = (dat == 0) ? State::CHECKSUM : State::DATA;
        break;
      case State::DATA:
        data_.push_back(dat);
        if (++recv_count_ >= length_) {state_ = State::CHECKSUM;}
        break;
      case State::CHECKSUM: {
          // CRC 는 [func, length, data...] 에 대해 계산
          std::vector<uint8_t> frame = {static_cast<uint8_t>(func_), length_};
          frame.insert(frame.end(), data_.begin(), data_.end());
          if (checksum_crc8(frame.data(), frame.size()) == dat) {
            cb_(func_, data_);
          }
          state_ = State::START1;
          break;
        }
    }
  }

private:
  enum class State {START1, START2, FUNCTION, LENGTH, DATA, CHECKSUM};
  State state_{State::START1};
  PacketFunction func_{PacketFunction::NONE};
  uint8_t length_{0};
  uint8_t recv_count_{0};
  std::vector<uint8_t> data_;
  Callback cb_;
};

// ── 수신 데이터 해석 (파이썬 get_* 와 동일) ──

// SYS 응답: [0x04, 전압(mV) uint16]
inline std::optional<uint16_t> decode_battery(const std::vector<uint8_t> & d)
{
  if (d.size() >= 3 && d[0] == 0x04) {
    return get_u16(&d[1]);
  }
  return std::nullopt;
}

// KEY 응답: [id, event] → (id, 0=클릭/1=눌림), 그 외 이벤트는 무시
inline std::optional<std::pair<uint8_t, uint8_t>> decode_button(const std::vector<uint8_t> & d)
{
  if (d.size() < 2) {return std::nullopt;}
  if (d[1] == 0x20) {return std::make_pair(d[0], uint8_t{0});}  // CLICK
  if (d[1] == 0x01) {return std::make_pair(d[0], uint8_t{1});}  // PRESSED
  return std::nullopt;
}

// IMU 응답: '<6f' = ax, ay, az, gx, gy, gz
inline std::optional<std::array<float, 6>> decode_imu(const std::vector<uint8_t> & d)
{
  if (d.size() < 24) {return std::nullopt;}
  std::array<float, 6> out;
  for (int i = 0; i < 6; ++i) {
    out[i] = get_f32(&d[i * 4]);
  }
  return out;
}

struct GamepadData
{
  std::array<float, 8> axes{};      // lx, ly, rx, ry, r2, l2, hat_x, hat_y
  std::array<int32_t, 16> buttons{};
};

// GAMEPAD 응답: '<HB4b' = 버튼마스크, hat, lx, ly, rx, ry
inline std::optional<GamepadData> decode_gamepad(const std::vector<uint8_t> & d)
{
  if (d.size() < 7) {return std::nullopt;}
  GamepadData g;
  const uint16_t mask = get_u16(&d[0]);
  const uint8_t hat = d[2];
  const int8_t sticks[4] = {
    static_cast<int8_t>(d[3]), static_cast<int8_t>(d[4]),
    static_cast<int8_t>(d[5]), static_cast<int8_t>(d[6])};

  if (mask & 0x0002) {g.axes[4] = 1.0f;}   // R2
  if (mask & 0x0001) {g.axes[5] = 1.0f;}   // L2
  if (mask & 0x0100) {g.buttons[0] = 1;}   // CROSS
  if (mask & 0x0200) {g.buttons[1] = 1;}   // CIRCLE
  if (mask & 0x0800) {g.buttons[3] = 1;}   // SQUARE
  if (mask & 0x1000) {g.buttons[4] = 1;}   // TRIANGLE
  if (mask & 0x4000) {g.buttons[6] = 1;}   // L1
  if (mask & 0x8000) {g.buttons[7] = 1;}   // R1
  if (mask & 0x0004) {g.buttons[10] = 1;}  // SELECT
  if (mask & 0x0008) {g.buttons[11] = 1;}  // START

  // 스틱: 양수는 /127, 음수는 /128 (lx, rx 는 부호 반전) — 파이썬과 동일
  const int signs[4] = {-1, 1, -1, 1};
  for (int i = 0; i < 4; ++i) {
    const int v = sticks[i];
    if (v > 0) {
      g.axes[i] = signs[i] * v / 127.0f;
    } else if (v < 0) {
      g.axes[i] = signs[i] * v / 128.0f;
    }
  }

  if (hat == 9) {g.axes[6] = 1.0f;} else if (hat == 13) {g.axes[6] = -1.0f;}
  if (hat == 11) {g.axes[7] = -1.0f;} else if (hat == 15) {g.axes[7] = 1.0f;}
  return g;
}

// SBUS 응답: '<16hBBBB' → 채널값을 -1~1 로 정규화 (신호 유실 시 파이썬과 동일한 기본값)
inline std::optional<std::vector<float>> decode_sbus(const std::vector<uint8_t> & d)
{
  if (d.size() < 36) {return std::nullopt;}
  const uint8_t signal_loss = d[34];
  std::vector<float> out;
  if (signal_loss != 0) {
    out.assign(16, 0.5f);
    out[4] = out[5] = out[6] = out[7] = 0.0f;
  } else {
    out.reserve(16);
    for (int i = 0; i < 16; ++i) {
      const int16_t ch = get_i16(&d[i * 2]);
      out.push_back(2.0f * (ch - 192) / (1792.0f - 192.0f) - 1.0f);
    }
  }
  return out;
}

}  // namespace ros_robot_controller_cpp

#endif  // ROS_ROBOT_CONTROLLER_CPP__BOARD_PROTOCOL_HPP_
