// stub_engine.cpp
// ─────────────────────────────────────────────────────────────────────────────
// PuppyEngine 의 임시(스텁) 구현.
//
// ★ 아직 실제 보행 엔진이 아니다 — 하드웨어(서보) 출력이 전혀 없다. ★
// 실제 구현은 사용자가 보유한 Hiwonder system image를 로컬에서 참고한다.
// 아래 파일과 포팅 구현은 저장소에 포함하거나 재배포하지 않는다:
//   /home/ubuntu/software/puppypi_control/puppy_kinematics.py  (HiwonderPuppy: 보행/IK 수학)
//   /home/ubuntu/software/puppypi_control/pwm_servo_control.py (서보 펄스 출력)
//   /home/ubuntu/software/puppypi_control/servo_controller.py
//   /home/ubuntu/software/puppypi_control/action_group_control.py (.d6a 동작 재생)
// 비공개 구현은 .gitignore 대상인 private/src/에 둔다.
//
// 스텁의 역할:
//  - puppy_control_node(ROS 래퍼)가 빌드·실행되어 토픽/서비스 인터페이스를
//    실기기 없이 검증할 수 있게 한다
//  - stance/gait 상태를 내부에 저장해 get_coord() 가 그럴듯한 값을 반환하게 한다
// ─────────────────────────────────────────────────────────────────────────────

#include <memory>
#include <string>

#include "puppy_control_cpp/puppy_engine.hpp"
#include "rclcpp/rclcpp.hpp"

namespace puppy_control_cpp
{

class StubPuppyEngine : public PuppyEngine
{
public:
  StubPuppyEngine()
  {
    RCLCPP_WARN(
      rclcpp::get_logger("puppy_engine"),
      "★ 보행 엔진 미이식 상태(StubPuppyEngine) — 서보 출력 없음. "
      "사용자가 보유한 순정 이미지 기반의 로컬 private 엔진이 있어야 "
      "로봇이 움직입니다.");
  }

  void stance_config(const Mat34 & stance, double pitch, double roll) override
  {
    stance_ = stance;
    pitch_ = pitch;
    roll_ = roll;
    log_once("stance_config");
  }

  void gait_config(
    double overlap_time, double swing_time,
    double clearance_time, double z_clearance) override
  {
    overlap_time_ = overlap_time;
    swing_time_ = swing_time;
    clearance_time_ = clearance_time;
    z_clearance_ = z_clearance;
    log_once("gait_config");
  }

  void start() override {log_once("start");}
  void end() override {log_once("end");}

  void move(double x, double y, double yaw_rate) override
  {
    x_ = x;
    y_ = y;
    yaw_rate_ = yaw_rate;
    log_once("move");
  }

  void move_stop(int servo_run_time_ms) override
  {
    (void)servo_run_time_ms;
    x_ = y_ = yaw_rate_ = 0.0;
    log_once("move_stop");
  }

  void servo_force_run() override {log_once("servo_force_run");}

  // 현재 stance 목표를 그대로 반환 (실제 엔진은 보행 위상에 따른 실시간 발끝 좌표)
  Mat34 get_coord() override {return stance_;}

  Mat34 four_legs_relative_coord_control(const Mat34 & foot_locations) override
  {
    (void)foot_locations;
    log_once("four_legs_relative_coord_control");
    return zero_mat34();  // TODO(엔진 이식): 역기구학 계산
  }

  void send_servo_angle(const Mat34 & joint_angles) override
  {
    (void)joint_angles;
    log_once("send_servo_angle");
  }

  void set_imu(ImuSource imu) override {imu_ = std::move(imu);}

  void set_servo_pulse(int id, int pulse, int time_ms) override
  {
    (void)id;
    (void)pulse;
    (void)time_ms;
    log_once("set_servo_pulse");
  }

  void run_action_group(const std::string & name, bool wait) override
  {
    (void)wait;
    RCLCPP_WARN(
      rclcpp::get_logger("puppy_engine"),
      "run_action_group('%s') 호출됨 — 엔진 미이식으로 무시", name.c_str());
  }

private:
  void log_once(const char * fn)
  {
    // 함수별 첫 호출만 알림 (도배 방지) — 인터페이스가 살아있음을 확인하는 용도
    RCLCPP_INFO_ONCE(
      rclcpp::get_logger("puppy_engine"),
      "엔진 호출 확인: %s(...) — 스텁이라 하드웨어 출력은 없음", fn);
  }

  Mat34 stance_{zero_mat34()};
  double pitch_{0.0}, roll_{0.0};
  double overlap_time_{0.1}, swing_time_{0.15}, clearance_time_{0.0}, z_clearance_{5.0};
  double x_{0.0}, y_{0.0}, yaw_rate_{0.0};
  ImuSource imu_;
};

std::unique_ptr<PuppyEngine> make_puppy_engine()
{
  return std::make_unique<StubPuppyEngine>();
}

}  // namespace puppy_control_cpp
