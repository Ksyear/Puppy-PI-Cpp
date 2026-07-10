// vr_teleop_logic.hpp
// ─────────────────────────────────────────────────────────────────────────────
// vr_udp_teleop 의 순수 로직 (ROS 의존 없음 → colcon test 로 단독 검증).
// 프로토콜 원본: PuppyPi_MR_Controller/raspberry_pi/udp_joystick_receiver.py
// ─────────────────────────────────────────────────────────────────────────────
#ifndef PUPPY_VR_CONTROL__VR_TELEOP_LOGIC_HPP_
#define PUPPY_VR_CONTROL__VR_TELEOP_LOGIC_HPP_

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <utility>

namespace puppy_vr_control
{

// "X:12.3,Z:-5.0" → (x, z). 형식이 깨졌으면 std::nullopt (예외로 죽지 않음).
// 원본 파이썬 parse_packet() 과 동일 규칙: 콤마 분리, 콜론 없는 조각 무시,
// 대소문자 무시, X/Z 둘 다 있어야 유효.
inline std::optional<std::pair<float, float>> parse_packet(const std::string & text)
{
  auto trim = [](const std::string & s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) {
      return std::string();
    }
    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
  };

  bool has_x = false;
  bool has_z = false;
  float x = 0.0f;
  float z = 0.0f;

  size_t pos = 0;
  while (pos <= text.size()) {
    const size_t comma = text.find(',', pos);
    const std::string part =
      text.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);

    const size_t colon = part.find(':');
    if (colon != std::string::npos) {
      const std::string key = trim(part.substr(0, colon));
      const std::string val = trim(part.substr(colon + 1));
      try {
        const float f = std::stof(val);
        if (key == "X" || key == "x") {
          x = f;
          has_x = true;
        } else if (key == "Z" || key == "z") {
          z = f;
          has_z = true;
        }
      } catch (const std::exception &) {
        return std::nullopt;  // 숫자 변환 실패 → 패킷 전체 무효
      }
    }

    if (comma == std::string::npos) {
      break;
    }
    pos = comma + 1;
  }

  if (!has_x || !has_z) {
    return std::nullopt;
  }
  return std::make_pair(x, z);
}

// 데드존 + 정규화 (원본 to_velocity 의 norm 과 동일):
// |angle| < deadzone 이면 0, 그 외 angle/max_angle 을 -1~1 로 클램프
inline double normalize_angle(double angle, double deadzone_deg, double max_angle_deg)
{
  if (std::abs(angle) < deadzone_deg) {
    return 0.0;
  }
  return std::clamp(angle / max_angle_deg, -1.0, 1.0);
}

// 양자화: step 단위로 반올림 (gait 재계산 억제용)
inline double quantize(double value, double step)
{
  return std::round(value / step) * step;
}

}  // namespace puppy_vr_control

#endif  // PUPPY_VR_CONTROL__VR_TELEOP_LOGIC_HPP_
