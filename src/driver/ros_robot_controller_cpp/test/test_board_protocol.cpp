// test_board_protocol.cpp — 시리얼 프로토콜 검증 (colcon test 로 실행)
//
// 아래 기준(expected) 바이트/값은 2026-07-09 에 파이썬 원본 SDK
// (ros_robot_controller_sdk.py 에 가짜 serial 을 주입해 write 캡처)로 생성해
// C++ 출력과 완전 일치를 확인한 것이다. 이 테스트는 그 기준을 고정해
// 이후 코드 수정이 프로토콜을 깨지 않는지 회귀 검사한다.
#include <cassert>
#include <cstdio>
#include <string>
#include <vector>

#include "ros_robot_controller_cpp/board_protocol.hpp"

using namespace ros_robot_controller_cpp;

static std::string to_hex(const std::vector<uint8_t> & f)
{
  std::string out;
  char b[3];
  for (uint8_t c : f) {
    std::snprintf(b, sizeof(b), "%02x", c);
    out += b;
  }
  return out;
}

static void expect(const char * name, const std::vector<uint8_t> & frame, const char * hex)
{
  const std::string got = to_hex(frame);
  if (got != hex) {
    std::printf("불일치 %s:\n  기대: %s\n  실제: %s\n", name, hex, got.c_str());
    assert(false);
  }
}

int main()
{
  // ── 명령 프레임 == 파이썬 SDK 출력 (대조 검증 완료된 기준값) ──
  expect("set_led", frame_set_led(0.1, 0.9, 1, 1), "aa55010701640084030100a1");
  expect("set_buzzer", frame_set_buzzer(1900, 0.1, 0.9, 1), "aa5502086c076400840301005d");
  expect("set_motor_speed", frame_set_motor_speed({{1, 0.5}, {2, -0.25}, {3, 0}, {4, 1.5}}),
    "aa5503160104000000003f01000080be0200000000030000c03f5a");
  expect("set_oled_text", frame_set_oled_text(1, "Hello"), "aa550a07010548656c6c6fe1");
  expect("set_rgb", frame_set_rgb({{{1, 255, 0, 128}}, {{2, 10, 20, 30}}}),
    "aa550b0a010200ff0080010a141e7c");
  expect("pwm_set_position", frame_pwm_servo_set_position(0.5, {{1, 1500}, {2, 2000}}),
    "aa55040a01f4010201dc0502d0075c");
  expect("pwm_set_offset", frame_pwm_servo_set_offset(3, -10), "aa5504030703f611");
  expect("bus_set_position", frame_bus_servo_set_position(1.234, {{1, 500}, {2, 0}}),
    "aa55050a01d2040201f401020000ef");
  expect("bus_torque_on", frame_bus_servo_enable_torque(1, true), "aa5505020b01b3");
  expect("bus_torque_off", frame_bus_servo_enable_torque(1, false), "aa5505020c01dd");
  expect("bus_set_id", frame_bus_servo_set_id(254, 1), "aa55050310fe010b");
  expect("bus_set_offset", frame_bus_servo_set_offset(1, -10), "aa5505032001f6a3");
  expect("bus_save_offset", frame_bus_servo_save_offset(1), "aa55050224016a");
  expect("bus_angle_limit", frame_bus_servo_set_angle_limit(1, 0, 1000), "aa55050630010000e80319");
  expect("bus_vin_limit", frame_bus_servo_set_vin_limit(1, 4500, 14500), "aa55050634019411a43883");
  expect("bus_temp_limit", frame_bus_servo_set_temp_limit(1, 85), "aa55050338015581");
  expect("bus_stop", frame_bus_servo_stop({1, 2}), "aa550504030201024b");
  expect("pwm_read_pos_req",
    frame_read_request(PacketFunction::PWM_SERVO, 0x05, 1), "aa5504020501e0");
  expect("pwm_read_off_req",
    frame_read_request(PacketFunction::PWM_SERVO, 0x09, 3), "aa550402090311");
  expect("bus_read_pos_req",
    frame_read_request(PacketFunction::BUS_SERVO, 0x05, 1), "aa55050205016f");
  expect("bus_read_id_req",
    frame_read_request(PacketFunction::BUS_SERVO, 0x12, 254), "aa55050212fed8");

  // ── CRC8 기준값 (파이썬 checksum_crc8 과 대조 완료) ──
  {
    std::vector<uint8_t> all;
    for (int i = 0; i < 256; ++i) {all.push_back(static_cast<uint8_t>(i));}
    assert(checksum_crc8(all.data(), all.size()) == 24);
    const uint8_t zero[1] = {0x00};
    assert(checksum_crc8(zero, 1) == 0);
    const std::vector<uint8_t> ff(16, 0xFF);
    assert(checksum_crc8(ff.data(), ff.size()) == 123);
    const std::string s = "PuppyPi VR";
    assert(checksum_crc8(reinterpret_cast<const uint8_t *>(s.data()), s.size()) == 190);
  }

  // ── 수신 디코드 ──
  {
    std::vector<uint8_t> d = {0x04};
    put_u16(d, 7400);
    assert(*decode_battery(d) == 7400);

    std::vector<uint8_t> imu;
    for (float f : {0.01f, -0.02f, 0.98f, 1.5f, -2.5f, 3.25f}) {put_f32(imu, f);}
    assert((*decode_imu(imu))[5] == 3.25f);

    assert(decode_button({1, 0x20})->second == 0);   // CLICK
    assert(decode_button({2, 0x01})->second == 1);   // PRESSED

    std::vector<uint8_t> gp;
    put_u16(gp, 0x0002 | 0x0100 | 0x0008);  // R2 | CROSS | START
    put_u8(gp, 9);
    put_i8(gp, 10); put_i8(gp, -20); put_i8(gp, 127); put_i8(gp, -128);
    const auto g = *decode_gamepad(gp);
    assert(g.axes[4] == 1.0f && g.buttons[0] == 1 && g.buttons[11] == 1);
    assert(g.axes[6] == 1.0f);                       // hat=9 → 위
  }

  // ── 파서 왕복: 쓰레기 + 정상 + CRC 오염 + 정상 → 정상 2개만 통과 ──
  {
    int good = 0;
    FrameParser parser(
      [&](PacketFunction func, const std::vector<uint8_t> & data) {
        if (func == PacketFunction::SYS && decode_battery(data)) {++good;}
      });
    std::vector<uint8_t> payload = {0x04};
    put_u16(payload, 7400);
    const auto frame = build_frame(PacketFunction::SYS, payload);
    std::vector<uint8_t> stream = {0x00, 0xAA, 0x12, 0xAA, 0x55, 0xFF};
    stream.insert(stream.end(), frame.begin(), frame.end());
    auto bad = frame;
    bad.back() ^= 0xFF;
    stream.insert(stream.end(), bad.begin(), bad.end());
    stream.insert(stream.end(), frame.begin(), frame.end());
    for (uint8_t b : stream) {parser.feed(b);}
    assert(good == 2);
  }

  std::printf("test_board_protocol: 전부 통과 (파이썬 SDK 기준값과 일치)\n");
  return 0;
}
