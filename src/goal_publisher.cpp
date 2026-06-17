// goal_publisher.cpp
// Second robotics project  -- goal publisher node
//
// Function: read a sequence of goals (x, y, theta) from a CSV file and send
// them to the robot one by one through Nav2's NavigateToPose action.
// A new goal is sent only after the current goal succeeds or is aborted.
//
// Key points:
//   - Uses an action client, not a topic, matching the "using action" requirement.
//   - Sequential state machine: only one active goal is tracked at a time.
//   - theta is yaw in radians and is converted to a quaternion for PoseStamped.
//   - The CSV path is passed as a parameter instead of being hard-coded.

#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <cctype>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// Simple goal point structure.
struct Goal {
  double x;
  double y;
  double theta;  // yaw in radians
};

class GoalPublisher : public rclcpp::Node
{
public:
  GoalPublisher() : Node("goal_publisher"), current_goal_index_(0)
  {
    // ---- Declare parameters ----
    this->declare_parameter<std::string>("csv_path", "goals.csv");
    this->declare_parameter<std::string>("goal_frame", "map");

    csv_path_   = this->get_parameter("csv_path").as_string();
    goal_frame_ = this->get_parameter("goal_frame").as_string();

    // ---- Create action client ----
    action_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    // ---- Load CSV ----
    if (!load_goals_from_csv(csv_path_)) {
      RCLCPP_ERROR(this->get_logger(),
        "Failed to read goal CSV file: %s", csv_path_.c_str());
      init_ok_ = false;
      return;
    }
    RCLCPP_INFO(this->get_logger(),
      "Loaded %zu goals from CSV", goals_.size());

    start_timer_ = this->create_wall_timer(
      1s, std::bind(&GoalPublisher::start_sending, this));
  }

  bool is_ok() const { return init_ok_; }

private:
  // -------------------- Load CSV --------------------
  bool load_goals_from_csv(const std::string & path)
  {
    std::ifstream file(path);
    if (!file.is_open()) {
      return false;
    }

    std::string line;
    bool first_line = true;
    while (std::getline(file, line)) {
      if (first_line) {
        first_line = false;
        if (!line.empty() && (std::isalpha(static_cast<unsigned char>(line[0])))) {
          continue;
        }
      }
      if (line.empty()) continue;

      std::stringstream ss(line);
      std::string token;
      Goal g;
      try {
        std::getline(ss, token, ','); g.x     = std::stod(token);
        std::getline(ss, token, ','); g.y     = std::stod(token);
        std::getline(ss, token, ','); g.theta = std::stod(token);
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(),
          "Skipping malformed line: %s", line.c_str());
        continue;
      }
      goals_.push_back(g);
    }
    file.close();
    return !goals_.empty();
  }

  // -------------------- Startup: wait for the server and send the first goal --------------------
  void start_sending()
  {
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Waiting for nav2 navigate_to_pose action server...");
      return;
    }

    start_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Nav2 action server is ready");
    send_next_goal();
  }

  // -------------------- Send the next goal --------------------
  void send_next_goal()
  {
    if (current_goal_index_ >= goals_.size()) {
      RCLCPP_INFO(this->get_logger(),
        "All %zu goals have been processed; shutting down", goals_.size());
      rclcpp::shutdown();
      return;
    }

    const Goal & g = goals_[current_goal_index_];

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = goal_frame_;
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = g.x;
    goal_msg.pose.pose.position.y = g.y;
    goal_msg.pose.pose.position.z = 0.0;

    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = std::sin(g.theta / 2.0);
    goal_msg.pose.pose.orientation.w = std::cos(g.theta / 2.0);

    RCLCPP_INFO(this->get_logger(),
      "Sending goal %zu/%zu: x=%.2f y=%.2f theta=%.3f",
      current_goal_index_ + 1, goals_.size(), g.x, g.y, g.theta);

    auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&GoalPublisher::goal_response_callback, this,
                std::placeholders::_1);

    send_goal_options.feedback_callback =
      std::bind(&GoalPublisher::feedback_callback, this,
                std::placeholders::_1, std::placeholders::_2);

    send_goal_options.result_callback =
      std::bind(&GoalPublisher::result_callback, this,
                std::placeholders::_1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  // -------------------- Callback: goal accepted/rejected --------------------
  void goal_response_callback(const GoalHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_WARN(this->get_logger(),
        "Goal %zu was rejected, retrying the same goal",
        current_goal_index_ + 1);
      start_timer_ = this->create_wall_timer(
        2s, std::bind(&GoalPublisher::retry_current_goal, this));
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted; robot is moving");
    }
  }

  void retry_current_goal()
  {
    start_timer_->cancel();
    send_next_goal();
  }

  // -------------------- Callback: navigation feedback --------------------
  void feedback_callback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    static int counter = 0;
    if (counter++ % 20 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "  Distance remaining: %.2f m", feedback->distance_remaining);
    }
  }

  // -------------------- Callback: goal finished (succeeded/aborted/canceled) --------------------
  void result_callback(const GoalHandle::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(),
          "Goal %zu reached", current_goal_index_ + 1);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(this->get_logger(),
          "Goal %zu was aborted; continuing to the next goal",
          current_goal_index_ + 1);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(),
          "Goal %zu was canceled", current_goal_index_ + 1);
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }

    current_goal_index_++;
    send_next_goal();
  }

  // -------------------- Member variables --------------------
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  std::vector<Goal> goals_;
  size_t current_goal_index_;
  std::string csv_path_;
  std::string goal_frame_;
  bool init_ok_ = true;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalPublisher>();
  if (node->is_ok()) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
