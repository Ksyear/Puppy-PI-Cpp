#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ros1_maze_escape/frontier_detector.hpp"
#include "ros1_maze_escape/frontier_scorer.hpp"

namespace ros1_maze_escape {

enum class MazeState {
  IDLE,
  CHECK_SYSTEM,
  SELECT_FRONTIER,
  REQUEST_GLOBAL_PLAN,
  NAVIGATING,
  CHECK_EXIT,
  RECOVERY,
  FINISHED,
  ERROR,
  EMERGENCY_STOP
};

struct FrontierSiteStats {
  geometry_msgs::Point location;
  unsigned int visit_count{0U};
  unsigned int failure_count{0U};
};

class MazeBrain {
 public:
  MazeBrain();

 private:
  using MoveBaseClient =
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& message);
  void emergencyStopCallback(const std_msgs::Bool::ConstPtr& message);
  void stateTimerCallback(const ros::TimerEvent&);

  void handleCheckSystem();
  void handleSelectFrontier();
  void handlePlanRequest();
  void handleNavigation();
  void handleCheckExit();
  void handleRecovery();

  bool lookupRobotPose(geometry_msgs::PoseStamped* pose);
  bool requestPlan(FrontierCandidate* candidate);
  FrontierScore scoreCandidate(const FrontierFeatures& features) const;
  void beginNavigation(const FrontierCandidate& candidate);
  void recordNavigationResult(bool succeeded);
  bool isBlacklisted(const geometry_msgs::Point& point) const;
  FrontierSiteStats* findOrCreateSite(const geometry_msgs::Point& point);
  const FrontierSiteStats* findSite(const geometry_msgs::Point& point) const;
  void transitionTo(MazeState next_state, const std::string& reason);
  void cancelNavigation();

  static double pathLength(const nav_msgs::Path& path,
                           const geometry_msgs::PoseStamped& start);
  static const char* stateName(MazeState state);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber map_sub_;
  ros::Subscriber emergency_stop_sub_;
  ros::Publisher state_pub_;
  ros::ServiceClient make_plan_client_;
  ros::Timer state_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<MoveBaseClient> move_base_client_;
  std::unique_ptr<FrontierDetector> frontier_detector_;
  std::unique_ptr<HeuristicFrontierScorer> heuristic_scorer_;

  nav_msgs::OccupancyGrid latest_map_;
  bool have_map_{false};
  bool emergency_stop_active_{false};
  bool goal_active_{false};
  mutable bool warned_ai_unavailable_{false};
  MazeState state_{MazeState::IDLE};

  std::string map_topic_;
  std::string emergency_stop_topic_;
  std::string make_plan_service_;
  std::string move_base_action_;
  std::string map_frame_;
  std::string robot_base_frame_;
  double state_update_rate_hz_{2.0};
  double make_plan_tolerance_m_{0.10};
  double navigation_goal_timeout_s_{120.0};
  double blacklist_radius_m_{0.35};
  int failure_blacklist_threshold_{2};
  int max_candidates_per_cycle_{30};
  int no_frontier_cycles_required_{3};
  bool use_ai_scorer_{false};
  int ai_timeout_ms_{50};

  geometry_msgs::PoseStamped robot_pose_;
  std::vector<FrontierCandidate> candidates_;
  std::size_t candidate_index_{0U};
  bool have_best_candidate_{false};
  FrontierCandidate best_candidate_;
  double best_score_{0.0};
  FrontierCandidate active_candidate_;
  ros::SteadyTime goal_start_time_;
  int consecutive_no_frontier_cycles_{0};
  std::vector<FrontierSiteStats> site_stats_;
};

}  // namespace ros1_maze_escape
