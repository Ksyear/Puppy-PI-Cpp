// puppy_engine.hpp
// ─────────────────────────────────────────────────────────────────────────────
// 보행/IK 엔진 인터페이스.
//
// 파이썬 puppy_control(puppy.py)은 로봇 내부 /home/ubuntu/software/puppypi_control
// 의 외부 모듈들(HiwonderPuppy, servo_controller, pwm_servo_control,
// action_group_control)을 불러 쓴다. 이 모듈들은 공식 git 저장소에 없으며
// 재배포하지 않는다. C++ 이식에서는 그 경계를 이 추상 인터페이스로 분리하고,
// 실제 구현은 .gitignore 대상인 private/에만 둔다.
//
//   PUPPY 노드 (puppy_control_node.cpp, 완전 이식됨)
//        │  이 인터페이스 호출
//        ▼
//   PuppyEngine ─── StubPuppyEngine  : 현재 (하드웨어 출력 없음, 로그만)
//               └── PrivatePuppyEngine : 사용자 로컬 전용 실제 구현(공개하지 않음)
//
// 파이썬 원본 대응:
//   stance_config / gait_config / start / end / move / move_stop /
//   servo_force_run / get_coord / four_legs_relative_coord_control /
//   send_servo_angle  ← HiwonderPuppy 의 동명 메서드
//   set_servo_pulse   ← servo_controller.setServoPulse / PWMServoControl.setPulse
//   run_action_group  ← action_group_control.runActionGroup
// ─────────────────────────────────────────────────────────────────────────────
#ifndef PUPPY_CONTROL_CPP__PUPPY_ENGINE_HPP_
#define PUPPY_CONTROL_CPP__PUPPY_ENGINE_HPP_

#include <array>
#include <functional>
#include <string>

namespace puppy_control_cpp
{

// 3×4 행렬: 열 = [오른앞, 왼앞, 오른뒤, 왼뒤] 다리, 행 = x/y/z (cm)
using Mat34 = std::array<std::array<double, 4>, 3>;

inline Mat34 zero_mat34()
{
  return Mat34{{{{0, 0, 0, 0}}, {{0, 0, 0, 0}}, {{0, 0, 0, 0}}}};
}

// IMU 오일러각 공급자 (자기균형용). 파이썬 MPU6050.get_euler_angle() 대응.
struct EulerAngle
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};
using ImuSource = std::function<EulerAngle(double /*dt*/)>;

class PuppyEngine
{
public:
  virtual ~PuppyEngine() = default;

  // ── HiwonderPuppy 대응 ──
  // 발끝 목표 위치(3×4)와 몸통 pitch/roll 로 서있는 자세 구성
  virtual void stance_config(const Mat34 & stance, double pitch, double roll) = 0;
  // 보행 타이밍 설정 (단위: 초, z_clearance 는 cm)
  virtual void gait_config(
    double overlap_time, double swing_time,
    double clearance_time, double z_clearance) = 0;
  virtual void start() = 0;   // 보행 제어 루프 시작
  virtual void end() = 0;     // 보행 제어 루프 정지
  // 이동 명령 (x: cm/s 스케일, yaw_rate: rad/s)
  virtual void move(double x, double y, double yaw_rate) = 0;
  virtual void move_stop(int servo_run_time_ms) = 0;
  virtual void servo_force_run() = 0;
  // 현재 발끝 좌표 (3×4)
  virtual Mat34 get_coord() = 0;
  // 발끝 상대좌표(3×4) → 관절 각도(3×4, rad)
  virtual Mat34 four_legs_relative_coord_control(const Mat34 & foot_locations) = 0;
  virtual void send_servo_angle(const Mat34 & joint_angles) = 0;
  // 자기균형: IMU 각도 공급자 연결/해제 (nullptr = 해제)
  virtual void set_imu(ImuSource imu) = 0;

  // ── servo_controller / pwm_servo_control 대응 ──
  virtual void set_servo_pulse(int id, int pulse, int time_ms) = 0;

  // ── action_group_control 대응 ──
  virtual void run_action_group(const std::string & name, bool wait) = 0;
};

}  // namespace puppy_control_cpp

#endif  // PUPPY_CONTROL_CPP__PUPPY_ENGINE_HPP_
