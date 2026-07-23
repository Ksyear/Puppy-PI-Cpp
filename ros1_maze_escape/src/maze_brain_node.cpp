#include "ros1_maze_escape/maze_brain.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <tf2/exceptions.h>

namespace ros1_maze_escape {

MazeBrain::MazeBrain()
    : private_nh_("~"), tf_listener_(tf_buffer_) {
  private_nh_.param<std::string>("map_topic", map_topic_, "/map");
  private_nh_.param<std::string>(
      "emergency_stop_topic", emergency_stop_topic_, "/emergency_stop");
  private_nh_.param<std::string>(
      "make_plan_service", make_plan_service_, "/move_base/make_plan");
  private_nh_.param<std::string>(
      "move_base_action", move_base_action_, "/move_base");
  private_nh_.param<std::string>("map_frame", map_frame_, "map");
  private_nh_.param<std::string>(
      "robot_base_frame", robot_base_frame_, "base_footprint");
  private_nh_.param("state_update_rate_hz", state_update_rate_hz_, 2.0);
  private_nh_.param(
      "make_plan_tolerance_m", make_plan_tolerance_m_, 0.10);
  private_nh_.param(
      "navigation_goal_timeout_s", navigation_goal_timeout_s_, 120.0);
  private_nh_.param("blacklist_radius_m", blacklist_radius_m_, 0.35);
  private_nh_.param(
      "failure_blacklist_threshold", failure_blacklist_threshold_, 2);
  private_nh_.param(
      "max_candidates_per_cycle", max_candidates_per_cycle_, 30);
  private_nh_.param(
      "no_frontier_cycles_required", no_frontier_cycles_required_, 3);
  private_nh_.param("use_ai_scorer", use_ai_scorer_, false);
  private_nh_.param("ai_timeout_ms", ai_timeout_ms_, 50);

  FrontierDetectorConfig detector_config;
  private_nh_.param(
      "min_frontier_size_m", detector_config.min_frontier_size_m, 0.30);
  private_nh_.param("clearance_search_radius_m",
                    detector_config.clearance_search_radius_m, 0.60);

  HeuristicWeights weights;
  private_nh_.param(
      "weights/information_gain", weights.information_gain, 2.0);
  private_nh_.param(
      "weights/actual_path_length", weights.actual_path_length, -1.5);
  private_nh_.param("weights/clearance", weights.clearance, 1.5);
  private_nh_.param(
      "weights/heading_change", weights.heading_change, -0.4);
  private_nh_.param("weights/visit_count", weights.visit_count, -3.0);
  private_nh_.param("weights/failure_count", weights.failure_count, -5.0);
  private_nh_.param(
      "weights/exit_probability", weights.exit_probability, 3.0);

  const bool parameters_valid =
      std::isfinite(state_update_rate_hz_) && state_update_rate_hz_ > 0.0 &&
      std::isfinite(make_plan_tolerance_m_) && make_plan_tolerance_m_ >= 0.0 &&
      std::isfinite(navigation_goal_timeout_s_) &&
      navigation_goal_timeout_s_ > 0.0 &&
      std::isfinite(blacklist_radius_m_) && blacklist_radius_m_ > 0.0 &&
      failure_blacklist_threshold_ > 0 && max_candidates_per_cycle_ > 0 &&
      no_frontier_cycles_required_ > 0 && ai_timeout_ms_ > 0 &&
      std::isfinite(detector_config.min_frontier_size_m) &&
      detector_config.min_frontier_size_m > 0.0 &&
      std::isfinite(detector_config.clearance_search_radius_m) &&
      detector_config.clearance_search_radius_m > 0.0 &&
      std::isfinite(weights.information_gain) &&
      std::isfinite(weights.actual_path_length) &&
      std::isfinite(weights.clearance) &&
      std::isfinite(weights.heading_change) &&
      std::isfinite(weights.visit_count) &&
      std::isfinite(weights.failure_count) &&
      std::isfinite(weights.exit_probability);
  if (!parameters_valid) {
    throw std::invalid_argument("maze_brain parameters are invalid");
  }

  frontier_detector_ =
      std::make_unique<FrontierDetector>(detector_config);
  heuristic_scorer_ =
      std::make_unique<HeuristicFrontierScorer>(weights);
  move_base_client_ =
      std::make_unique<MoveBaseClient>(move_base_action_, true);
  make_plan_client_ =
      nh_.serviceClient<nav_msgs::GetPlan>(make_plan_service_, false);

  map_sub_ = nh_.subscribe(
      map_topic_, 1, &MazeBrain::mapCallback, this);
  emergency_stop_sub_ = nh_.subscribe(
      emergency_stop_topic_, 10, &MazeBrain::emergencyStopCallback, this);
  state_pub_ = private_nh_.advertise<std_msgs::String>("state", 1, true);
  state_timer_ = nh_.createTimer(
      ros::Duration(1.0 / state_update_rate_hz_),
      &MazeBrain::stateTimerCallback, this);

  transitionTo(MazeState::CHECK_SYSTEM, "node initialized");
}

void MazeBrain::mapCallback(
    const nav_msgs::OccupancyGrid::ConstPtr& message) {
  if (message->info.width == 0U || message->info.height == 0U ||
      message->info.resolution <= 0.0 ||
      message->data.size() !=
          static_cast<std::size_t>(message->info.width) *
              static_cast<std::size_t>(message->info.height)) {
    ROS_ERROR_THROTTLE(2.0, "Ignoring malformed occupancy grid");
    return;
  }
  if (message->header.frame_id != map_frame_) {
    ROS_ERROR_THROTTLE(
        2.0, "Ignoring occupancy grid in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
    return;
  }
  latest_map_ = *message;
  have_map_ = true;
}

void MazeBrain::emergencyStopCallback(
    const std_msgs::Bool::ConstPtr& message) {
  emergency_stop_active_ = message->data;
  if (emergency_stop_active_) {
    cancelNavigation();
    transitionTo(MazeState::EMERGENCY_STOP, "emergency stop active");
  } else if (state_ == MazeState::EMERGENCY_STOP) {
    transitionTo(MazeState::CHECK_SYSTEM,
                 "emergency stop released; rechecking system");
  }
}

void MazeBrain::stateTimerCallback(const ros::TimerEvent&) {
  if (emergency_stop_active_) {
    if (state_ != MazeState::EMERGENCY_STOP) {
      cancelNavigation();
      transitionTo(MazeState::EMERGENCY_STOP, "emergency stop active");
    }
    return;
  }

  switch (state_) {
    case MazeState::IDLE:
      transitionTo(MazeState::CHECK_SYSTEM, "leaving idle");
      break;
    case MazeState::CHECK_SYSTEM:
      handleCheckSystem();
      break;
    case MazeState::SELECT_FRONTIER:
      handleSelectFrontier();
      break;
    case MazeState::REQUEST_GLOBAL_PLAN:
      handlePlanRequest();
      break;
    case MazeState::NAVIGATING:
      handleNavigation();
      break;
    case MazeState::CHECK_EXIT:
      handleCheckExit();
      break;
    case MazeState::RECOVERY:
      handleRecovery();
      break;
    case MazeState::FINISHED:
    case MazeState::ERROR:
    case MazeState::EMERGENCY_STOP:
      break;
  }
}

void MazeBrain::handleCheckSystem() {
  if (!have_map_) {
    ROS_WARN_THROTTLE(5.0, "Waiting for occupancy grid on %s",
                      map_topic_.c_str());
    return;
  }
  if (!make_plan_client_.exists()) {
    ROS_WARN_THROTTLE(5.0, "Waiting for service %s",
                      make_plan_service_.c_str());
    return;
  }
  if (!move_base_client_->isServerConnected()) {
    ROS_WARN_THROTTLE(5.0, "Waiting for action server %s",
                      move_base_action_.c_str());
    return;
  }
  if (!lookupRobotPose(&robot_pose_)) {
    return;
  }
  transitionTo(MazeState::SELECT_FRONTIER, "map, TF, planner, and move_base ready");
}

void MazeBrain::handleSelectFrontier() {
  if (!lookupRobotPose(&robot_pose_)) {
    transitionTo(MazeState::CHECK_SYSTEM, "robot pose unavailable");
    return;
  }

  candidates_ = frontier_detector_->detect(latest_map_, robot_pose_);
  candidates_.erase(
      std::remove_if(
          candidates_.begin(), candidates_.end(),
          [this](const FrontierCandidate& candidate) {
            return isBlacklisted(candidate.goal.pose.position);
          }),
      candidates_.end());
  if (candidates_.size() >
      static_cast<std::size_t>(max_candidates_per_cycle_)) {
    candidates_.resize(static_cast<std::size_t>(max_candidates_per_cycle_));
  }

  candidate_index_ = 0U;
  have_best_candidate_ = false;
  best_score_ = -std::numeric_limits<double>::infinity();
  if (candidates_.empty()) {
    ++consecutive_no_frontier_cycles_;
    transitionTo(MazeState::CHECK_EXIT,
                 "no non-blacklisted frontier candidates");
    return;
  }

  transitionTo(MazeState::REQUEST_GLOBAL_PLAN,
               "frontier candidates require reachability checks");
}

void MazeBrain::handlePlanRequest() {
  if (candidate_index_ < candidates_.size()) {
    FrontierCandidate& candidate = candidates_[candidate_index_++];
    if (!requestPlan(&candidate)) {
      return;
    }

    const FrontierSiteStats* stats = findSite(candidate.goal.pose.position);
    if (stats != nullptr) {
      candidate.features.visit_count = stats->visit_count;
      candidate.features.failure_count = stats->failure_count;
    }
    const FrontierScore score = scoreCandidate(candidate.features);
    if (score.valid &&
        (!have_best_candidate_ || score.value > best_score_)) {
      best_candidate_ = candidate;
      best_score_ = score.value;
      have_best_candidate_ = true;
    }
    ROS_DEBUG_STREAM("frontier plan score=" << score.value
                     << " valid=" << score.valid
                     << " source=" << score.source
                     << " path_length="
                     << candidate.features.actual_path_length);
    return;
  }

  if (!have_best_candidate_) {
    ++consecutive_no_frontier_cycles_;
    transitionTo(MazeState::CHECK_EXIT,
                 "no candidate produced a non-empty global plan");
    return;
  }

  consecutive_no_frontier_cycles_ = 0;
  beginNavigation(best_candidate_);
}

void MazeBrain::handleNavigation() {
  if (!goal_active_) {
    transitionTo(MazeState::RECOVERY, "navigation goal is not active");
    return;
  }

  if ((ros::SteadyTime::now() - goal_start_time_).toSec() >=
      navigation_goal_timeout_s_) {
    move_base_client_->cancelGoal();
    recordNavigationResult(false);
    transitionTo(MazeState::RECOVERY, "navigation goal timed out");
    return;
  }

  const actionlib::SimpleClientGoalState action_state =
      move_base_client_->getState();
  if (!action_state.isDone()) {
    return;
  }
  if (action_state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    recordNavigationResult(true);
    transitionTo(MazeState::CHECK_EXIT, "frontier goal reached");
  } else {
    const std::string reason =
        std::string("move_base ended with ") + action_state.toString();
    recordNavigationResult(false);
    transitionTo(MazeState::RECOVERY, reason);
  }
}

void MazeBrain::handleCheckExit() {
  // TODO(EXIT_DETECTOR): add a validated, maze-specific exit condition.
  // Until then, exploration is considered complete only after repeated cycles
  // produce no reachable, non-blacklisted frontier.
  if (consecutive_no_frontier_cycles_ >= no_frontier_cycles_required_) {
    cancelNavigation();
    transitionTo(MazeState::FINISHED,
                 "no reachable frontier remained for the configured cycles");
    return;
  }
  transitionTo(MazeState::SELECT_FRONTIER,
               "exit not yet established; continue exploration");
}

void MazeBrain::handleRecovery() {
  cancelNavigation();
  // move_base owns deterministic low-level recovery behaviors. This state only
  // reselects a goal after accounting for the failed site.
  transitionTo(MazeState::SELECT_FRONTIER,
               "failed site recorded; selecting another frontier");
}

bool MazeBrain::lookupRobotPose(geometry_msgs::PoseStamped* pose) {
  try {
    const geometry_msgs::TransformStamped transform =
        tf_buffer_.lookupTransform(
            map_frame_, robot_base_frame_, ros::Time(0), ros::Duration(0.05));
    pose->header = transform.header;
    pose->header.frame_id = map_frame_;
    pose->pose.position.x = transform.transform.translation.x;
    pose->pose.position.y = transform.transform.translation.y;
    pose->pose.position.z = transform.transform.translation.z;
    pose->pose.orientation = transform.transform.rotation;
    return true;
  } catch (const tf2::TransformException& error) {
    ROS_WARN_THROTTLE(
        2.0, "Cannot transform %s -> %s: %s",
        map_frame_.c_str(), robot_base_frame_.c_str(), error.what());
    return false;
  }
}

bool MazeBrain::requestPlan(FrontierCandidate* candidate) {
  nav_msgs::GetPlan service;
  service.request.start = robot_pose_;
  service.request.goal = candidate->goal;
  service.request.tolerance = make_plan_tolerance_m_;
  if (!make_plan_client_.call(service)) {
    ROS_ERROR_THROTTLE(2.0, "Call to %s failed",
                       make_plan_service_.c_str());
    transitionTo(MazeState::CHECK_SYSTEM,
                 "global planning service call failed");
    return false;
  }
  if (service.response.plan.poses.empty()) {
    FrontierSiteStats* stats =
        findOrCreateSite(candidate->goal.pose.position);
    ++stats->failure_count;
    ROS_WARN_STREAM("Rejected unreachable frontier at ("
                    << candidate->goal.pose.position.x << ", "
                    << candidate->goal.pose.position.y << ")");
    return false;
  }

  candidate->features.actual_path_length =
      pathLength(service.response.plan, robot_pose_);
  if (!std::isfinite(candidate->features.actual_path_length)) {
    return false;
  }
  return true;
}

FrontierScore MazeBrain::scoreCandidate(
    const FrontierFeatures& features) const {
  if (use_ai_scorer_ && !warned_ai_unavailable_) {
    warned_ai_unavailable_ = true;
    ROS_WARN_STREAM(
        "use_ai_scorer is true, but no ONNX backend is compiled. "
        "Using deterministic heuristic fallback; configured timeout is "
        << ai_timeout_ms_ << " ms");
  }

  // TODO(ONNX_SCORER): call an isolated scorer here with ai_timeout_ms_.
  // Accept its result only when it is timely and finite. Any exception,
  // timeout, NaN, or Inf must fall through to this deterministic scorer.
  return heuristic_scorer_->score(features);
}

void MazeBrain::beginNavigation(const FrontierCandidate& candidate) {
  active_candidate_ = candidate;
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = active_candidate_.goal;
  goal.target_pose.header.stamp = ros::Time::now();
  move_base_client_->sendGoal(goal);
  goal_start_time_ = ros::SteadyTime::now();
  goal_active_ = true;
  transitionTo(MazeState::NAVIGATING, "best reachable frontier sent to move_base");
}

void MazeBrain::recordNavigationResult(bool succeeded) {
  FrontierSiteStats* stats =
      findOrCreateSite(active_candidate_.goal.pose.position);
  if (succeeded) {
    ++stats->visit_count;
  } else {
    ++stats->failure_count;
  }
  goal_active_ = false;
}

bool MazeBrain::isBlacklisted(const geometry_msgs::Point& point) const {
  const FrontierSiteStats* stats = findSite(point);
  return stats != nullptr &&
         stats->failure_count >=
             static_cast<unsigned int>(failure_blacklist_threshold_);
}

FrontierSiteStats* MazeBrain::findOrCreateSite(
    const geometry_msgs::Point& point) {
  for (FrontierSiteStats& stats : site_stats_) {
    if (std::hypot(stats.location.x - point.x,
                   stats.location.y - point.y) <= blacklist_radius_m_) {
      return &stats;
    }
  }
  FrontierSiteStats stats;
  stats.location = point;
  site_stats_.push_back(stats);
  return &site_stats_.back();
}

const FrontierSiteStats* MazeBrain::findSite(
    const geometry_msgs::Point& point) const {
  for (const FrontierSiteStats& stats : site_stats_) {
    if (std::hypot(stats.location.x - point.x,
                   stats.location.y - point.y) <= blacklist_radius_m_) {
      return &stats;
    }
  }
  return nullptr;
}

void MazeBrain::transitionTo(
    MazeState next_state, const std::string& reason) {
  if (state_ == next_state) {
    return;
  }
  ROS_INFO_STREAM("maze_brain state " << stateName(state_) << " -> "
                  << stateName(next_state) << ": " << reason);
  state_ = next_state;
  std_msgs::String state_message;
  state_message.data = stateName(state_);
  state_pub_.publish(state_message);
}

void MazeBrain::cancelNavigation() {
  if (goal_active_) {
    move_base_client_->cancelGoal();
    goal_active_ = false;
  }
}

double MazeBrain::pathLength(
    const nav_msgs::Path& path,
    const geometry_msgs::PoseStamped& start) {
  if (path.poses.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double length = std::hypot(
      path.poses.front().pose.position.x - start.pose.position.x,
      path.poses.front().pose.position.y - start.pose.position.y);
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    length += std::hypot(
        path.poses[index].pose.position.x -
            path.poses[index - 1U].pose.position.x,
        path.poses[index].pose.position.y -
            path.poses[index - 1U].pose.position.y);
  }
  return length;
}

const char* MazeBrain::stateName(MazeState state) {
  switch (state) {
    case MazeState::IDLE:
      return "IDLE";
    case MazeState::CHECK_SYSTEM:
      return "CHECK_SYSTEM";
    case MazeState::SELECT_FRONTIER:
      return "SELECT_FRONTIER";
    case MazeState::REQUEST_GLOBAL_PLAN:
      return "REQUEST_GLOBAL_PLAN";
    case MazeState::NAVIGATING:
      return "NAVIGATING";
    case MazeState::CHECK_EXIT:
      return "CHECK_EXIT";
    case MazeState::RECOVERY:
      return "RECOVERY";
    case MazeState::FINISHED:
      return "FINISHED";
    case MazeState::ERROR:
      return "ERROR";
    case MazeState::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
  }
  return "ERROR";
}

}  // namespace ros1_maze_escape

int main(int argc, char** argv) {
  ros::init(argc, argv, "maze_brain");
  try {
    ros1_maze_escape::MazeBrain node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("maze_brain initialization failed: " << error.what());
    return 1;
  }
  return 0;
}
