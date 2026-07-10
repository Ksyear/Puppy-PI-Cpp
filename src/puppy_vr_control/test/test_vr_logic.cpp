// test_vr_logic.cpp — vr_udp_teleop 순수 로직 검증 (colcon test 로 실행)
// 기준값 출처: PuppyPi_MR_Controller 원본 파이썬 및 README 예시와 대조 완료
#include <cassert>
#include <cmath>
#include <cstdio>

#include "puppy_vr_control/vr_teleop_logic.hpp"

using namespace puppy_vr_control;

int main()
{
  // ── parse_packet: 원본 파이썬 parse_packet 과 동일 동작 ──
  auto r = parse_packet("X:12.3,Z:-5.0");
  assert(r && std::abs(r->first - 12.3f) < 1e-4 && std::abs(r->second + 5.0f) < 1e-4);

  r = parse_packet("X:10,Z:-10");
  assert(r && r->first == 10.0f && r->second == -10.0f);

  r = parse_packet(" x : 20.0 , z : -15.5 ");  // 공백/소문자 허용
  assert(r && r->first == 20.0f && std::abs(r->second + 15.5f) < 1e-4);

  assert(!parse_packet("X:12.3"));         // Z 없음 → 무효
  assert(!parse_packet("hello world"));    // 형식 아님 → 무효
  assert(!parse_packet("X:abc,Z:1"));      // 숫자 아님 → 무효
  assert(!parse_packet(""));               // 빈 패킷 → 무효
  assert(!parse_packet("ESTOP"));          // 명령 문자열은 좌표로 해석되지 않음
  r = parse_packet("X:1,Y:9,Z:2");         // 여분 키는 무시
  assert(r && r->first == 1.0f && r->second == 2.0f);

  // ── normalize_angle: 데드존/클램프 (원본 README 예시와 대조) ──
  // README: X=15.5, Z=-30.2 → 전진 -0.67, 회전 +0.34
  assert(std::abs(normalize_angle(-30.2, 5.0, 45.0) - (-0.671)) < 0.01);
  assert(std::abs(normalize_angle(15.5, 5.0, 45.0) - 0.344) < 0.01);
  assert(normalize_angle(4.9, 5.0, 45.0) == 0.0);     // 데드존
  assert(normalize_angle(90.0, 5.0, 45.0) == 1.0);    // 클램프
  assert(normalize_angle(-90.0, 5.0, 45.0) == -1.0);

  // ── quantize: 속도 양자화 ──
  assert(quantize(0.671 * 15.0, 1.0) == 10.0);
  assert(quantize(0.03 * 15.0, 1.0) == 0.0);          // 미세값 → 정지

  // ── 카메라 청크 분할 산술 (camera_udp_sender 와 동일 수식) ──
  {
    const size_t total = 45000, chunk = 1400;
    const size_t count = (total + chunk - 1) / chunk;
    assert(count == 33);
    size_t reassembled = 0;
    for (size_t i = 0; i < count; ++i) {
      reassembled += std::min(chunk, total - i * chunk);
    }
    assert(reassembled == total);   // 손실 없이 전체 복원
  }

  printf("test_vr_logic: 전부 통과\n");
  return 0;
}
