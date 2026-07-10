// obstacle_climb.cpp
// ─────────────────────────────────────────────────────────────────────────────
// OpenCV 로 전방 장애물(계단/턱)을 인식하고 PuppyPi 다리로 넘어가는 노드.
// 공식 negotiate_stairs_demo.py 의 C++ 이식 + 개선판.
//
// 동작 원리:
//   [탐색]  카메라(/image_raw)에서 LAB 색공간 임계값으로 장애물 마스크 추출
//           → 최대 윤곽선의 최소외접사각형 중심 (cx, cy) 계산
//   [접근]  cy 가 화면 아래쪽(trigger_y)에 올 때까지 전진.
//           ★개선: cx 와 화면 중앙의 오차에 비례한 yaw 보정으로 장애물에
//           정면 정렬 (원본은 정렬 없이 직진 → 비스듬히 오르다 넘어짐)
//   [등반]  정지 → /puppy_control/runActionGroup 으로 다리 동작 그룹 실행
//           (up_stairs_2cm.d6ac — 앞다리 들어올리기→앞다리 걸치기→뒷다리 밀기
//            순서의 사전 정의 다리 궤적. 동작 그룹은 로봇의 ActionGroups 폴더)
//   [통과]  동작 완료 후 잠깐 전진해 몸 전체를 넘긴 뒤 정지
//
// 안전 장치:
//   - 이동 중 영상이 image_timeout(2s) 이상 끊기면 즉시 정지
//   - 노드 종료 시 정지 명령 발행
//   - ~/set_running (SetBool) 로 언제든 중단/재시작
//
// 원본과 다른 점(의도적):
//   - cv2.imshow 대신 ~/debug_image 토픽으로 어노테이션 영상 발행 (헤드리스)
//   - 원본은 통과 후 x=14 전진 명령을 낸 채 영원히 걷는다 → pass_duration
//     뒤 정지하도록 수정
//   - 원본의 DOWN_STAIRS 진입 시 Pose(height=0.3) 발행은 puppy_control 이
//     범위 검사(-15~-5)로 조용히 무시하는 무효 명령이라 제거 (원본 버그)
// ─────────────────────────────────────────────────────────────────────────────

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc.hpp"
#include "puppy_control_msgs/msg/velocity.hpp"
#include "puppy_control_msgs/srv/set_run_action_name.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using Velocity = puppy_control_msgs::msg::Velocity;
using SetRunActionName = puppy_control_msgs::srv::SetRunActionName;

class ObstacleClimbNode : public rclcpp::Node
{
public:
  enum class State {LOOKING_FOR, CLIMBING, PASS_OVER, DONE};

  ObstacleClimbNode()
  : Node("obstacle_climb")
  {
    // ── 색 임계값 (LAB, OpenCV 0~255 스케일) — 기본값은 빨간 장애물 ──
    // 실제 값은 로봇의 /home/ubuntu/software/lab_tool/lab_config.yaml 에서
    // 캘리브레이션된 red 값을 확인해 파라미터로 넣을 것
    lab_min_ = declare_parameter<std::vector<int64_t>>("lab_min", {0, 150, 130});
    lab_max_ = declare_parameter<std::vector<int64_t>>("lab_max", {255, 255, 255});

    approach_speed_ = declare_parameter<double>("approach_speed", 10.0);   // cm/s
    pass_speed_ = declare_parameter<double>("pass_speed", 14.0);
    trigger_y_ = declare_parameter<int>("trigger_y", 400);      // 640x480 기준
    // 정렬 회전 속도 = gain × 정규화 중심오차(-1~1), 왼쪽에 있으면 +yaw(좌회전)
    align_yaw_gain_ = declare_parameter<double>("align_yaw_gain", 0.25);  // rad/s
    max_align_yaw_ = declare_parameter<double>("max_align_yaw", 0.25);    // rad/s
    min_area_ = declare_parameter<double>("min_area", 300.0);   // 320x240 기준 px^2

    climb_action_ = declare_parameter<std::string>("climb_action", "up_stairs_2cm.d6ac");
    climb_timeout_sec_ = declare_parameter<double>("climb_timeout_sec", 25.0);
    pass_duration_sec_ = declare_parameter<double>("pass_duration_sec", 3.0);
    image_timeout_sec_ = declare_parameter<double>("image_timeout_sec", 2.0);
    running_ = declare_parameter<bool>("autostart", true);

    velocity_pub_ = create_publisher<Velocity>("/puppy_control/velocity", 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug_image", 1);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&ObstacleClimbNode::image_cb, this, std::placeholders::_1));

    action_client_ = create_client<SetRunActionName>("/puppy_control/runActionGroup");

    set_running_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/set_running",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        running_ = req->data;
        if (!req->data) {
          publish_velocity(0.0, 0.0);
        } else {
          state_ = State::LOOKING_FOR;  // 재시작
        }
        res->success = true;
        res->message = running_ ? "running" : "stopped";
      });

    // 상태머신 + 영상 끊김 감시 (원본은 100Hz — 판단 로직에는 20Hz 로 충분)
    timer_ = create_wall_timer(50ms, std::bind(&ObstacleClimbNode::tick, this));

    RCLCPP_INFO(get_logger(),
      "장애물 넘기 시작: 트리거 y>%d, 동작그룹=%s", trigger_y_, climb_action_.c_str());
  }

  ~ObstacleClimbNode() override
  {
    publish_velocity(0.0, 0.0);  // 종료 시 안전 정지
  }

private:
  void publish_velocity(double x, double yaw)
  {
    Velocity msg;
    msg.x = static_cast<float>(x);
    msg.y = 0.0f;
    msg.yaw_rate = static_cast<float>(yaw);
    velocity_pub_->publish(msg);
  }

  // ── OpenCV: 원본 run() 과 동일 파이프라인 (320x240 축소 → LAB → 마스크 → 최대 윤곽) ──
  void image_cb(sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv;
    try {
      cv = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "이미지 변환 실패: %s", e.what());
      return;
    }
    last_image_time_ = now();

    const int img_w = cv->image.cols;
    const int img_h = cv->image.rows;
    cv::Mat small, blurred, lab, mask;
    cv::resize(cv->image, small, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);
    cv::GaussianBlur(small, blurred, cv::Size(3, 3), 3);
    cv::cvtColor(blurred, lab, cv::COLOR_BGR2Lab);
    cv::inRange(
      lab,
      cv::Scalar(lab_min_[0], lab_min_[1], lab_min_[2]),
      cv::Scalar(lab_max_[0], lab_max_[1], lab_max_[2]),
      mask);
    const cv::Mat kernel = cv::Mat::ones(6, 6, CV_8U);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_L1);

    double best_area = 0.0;
    int best = -1;
    for (size_t i = 0; i < contours.size(); ++i) {
      const double area = std::fabs(cv::contourArea(contours[i]));
      if (area > best_area) {
        best_area = area;
        best = static_cast<int>(i);
      }
    }

    target_found_ = false;
    if (best >= 0 && best_area >= min_area_) {
      const cv::RotatedRect rect = cv::minAreaRect(contours[best]);
      // 320x240 좌표 → 원본 해상도로 환산 (원본 Misc.map 대응)
      target_cx_ = static_cast<int>(rect.center.x * img_w / 320.0);
      target_cy_ = static_cast<int>(rect.center.y * img_h / 240.0);
      // 화면 중앙 대비 정규화 오차 (-1=오른쪽 끝, +1=왼쪽 끝)
      target_err_x_ = (img_w / 2.0 - target_cx_) / (img_w / 2.0);
      target_found_ = true;

      // 디버그 영상: 검출 사각형/중심 표시
      if (debug_pub_->get_subscription_count() > 0) {
        cv::Point2f box[4];
        rect.points(box);
        for (int i = 0; i < 4; ++i) {
          cv::line(
            cv->image,
            cv::Point(static_cast<int>(box[i].x * img_w / 320.0),
            static_cast<int>(box[i].y * img_h / 240.0)),
            cv::Point(static_cast<int>(box[(i + 1) % 4].x * img_w / 320.0),
            static_cast<int>(box[(i + 1) % 4].y * img_h / 240.0)),
            cv::Scalar(0, 0, 255), 2);
        }
        cv::circle(cv->image, cv::Point(target_cx_, target_cy_), 5, cv::Scalar(0, 0, 255), -1);
        debug_pub_->publish(*cv->toImageMsg());
      }
    }
  }

  // ── 상태머신 ──
  void tick()
  {
    if (!running_) {
      return;
    }

    // 안전: 이동 중 영상이 끊기면 즉시 정지
    if (state_ == State::LOOKING_FOR && last_image_time_.nanoseconds() > 0) {
      const double age = (now() - last_image_time_).seconds();
      if (age > image_timeout_sec_) {
        if (!image_lost_) {
          publish_velocity(0.0, 0.0);
          image_lost_ = true;
          RCLCPP_WARN(get_logger(), "영상 %.1fs 끊김 → 안전 정지", age);
        }
        return;
      }
      image_lost_ = false;
    }

    switch (state_) {
      case State::LOOKING_FOR: {
          if (target_found_ && target_cy_ > trigger_y_) {
            publish_velocity(0.0, 0.0);   // 장애물 바로 앞 → 정지
            start_climb();
          } else {
            // 접근: 장애물이 보이면 중앙 정렬하며 전진, 안 보이면 직진 탐색
            double yaw = 0.0;
            if (target_found_) {
              yaw = std::clamp(
                align_yaw_gain_ * target_err_x_, -max_align_yaw_, max_align_yaw_);
            }
            publish_velocity(approach_speed_, yaw);
          }
          break;
        }
      case State::CLIMBING: {
          // 동작 그룹 완료 대기 (응답 또는 타임아웃)
          const double elapsed = (now() - climb_start_).seconds();
          if (climb_done_ || elapsed > climb_timeout_sec_) {
            RCLCPP_INFO(get_logger(), "등반 완료(%.0fs) → 통과 전진", elapsed);
            pass_start_ = now();
            publish_velocity(pass_speed_, 0.0);
            state_ = State::PASS_OVER;
          }
          break;
        }
      case State::PASS_OVER: {
          if ((now() - pass_start_).seconds() >= pass_duration_sec_) {
            publish_velocity(0.0, 0.0);
            state_ = State::DONE;
            RCLCPP_INFO(get_logger(), "장애물 통과 완료 — 정지. 재시작: ~/set_running true");
          }
          break;
        }
      case State::DONE:
        break;
    }
  }

  void start_climb()
  {
    state_ = State::CLIMBING;
    climb_done_ = false;
    climb_start_ = now();

    if (!action_client_->wait_for_service(0s)) {
      RCLCPP_ERROR(get_logger(), "/puppy_control/runActionGroup 서비스 없음 — puppy_control 실행 확인");
      return;
    }
    auto req = std::make_shared<SetRunActionName::Request>();
    req->name = climb_action_;
    req->wait = true;
    RCLCPP_INFO(get_logger(), "다리 동작 그룹 실행: %s", climb_action_.c_str());
    action_client_->async_send_request(
      req, [this](rclcpp::Client<SetRunActionName>::SharedFuture future) {
        try {
          RCLCPP_INFO(get_logger(), "동작 그룹 응답: %s", future.get()->message.c_str());
          climb_done_ = true;
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "동작 그룹 호출 실패: %s", e.what());
          climb_done_ = true;  // 실패해도 상태머신은 진행 (정지 상태로 마무리됨)
        }
      });
  }

  // ── 상태 ──
  State state_{State::LOOKING_FOR};
  bool running_{true};
  bool target_found_{false};
  int target_cx_{0}, target_cy_{0};
  double target_err_x_{0.0};
  bool image_lost_{false};
  bool climb_done_{false};
  rclcpp::Time last_image_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time climb_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time pass_start_{0, 0, RCL_ROS_TIME};

  // ── 파라미터 ──
  std::vector<int64_t> lab_min_, lab_max_;
  double approach_speed_{10.0}, pass_speed_{14.0};
  int trigger_y_{400};
  double align_yaw_gain_{0.25}, max_align_yaw_{0.25};
  double min_area_{300.0};
  std::string climb_action_;
  double climb_timeout_sec_{25.0}, pass_duration_sec_{3.0}, image_timeout_sec_{2.0};

  // ── ROS 인터페이스 ──
  rclcpp::Publisher<Velocity>::SharedPtr velocity_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Client<SetRunActionName>::SharedPtr action_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_running_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleClimbNode>());
  rclcpp::shutdown();
  return 0;
}
