// goal_publisher.cpp
// 第二个机器人项目 (Task 2) —— 目标发布节点
//
// 功能:从 CSV 文件读取一系列目标 (x, y, theta),通过 nav2 的
//       NavigateToPose action 依次发送给机器人。
//       只有当前一个目标【到达】或【被中止】后,才发送下一个目标。
//
// 关键点:
//   - 使用 action(不是 topic),符合作业要求 "using action"
//   - 顺序状态机:一次只追踪一个目标
//   - theta (yaw 弧度) 转四元数填入 PoseStamped
//   - CSV 路径通过参数传入,绝对【不用绝对路径】(否则 0 分)

#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// 一个目标点的简单结构体
struct Goal {
  double x;
  double y;
  double theta;  // yaw,弧度
};

class GoalPublisher : public rclcpp::Node
{
public:
  GoalPublisher() : Node("goal_publisher"), current_goal_index_(0)
  {
    // ---- 声明参数 ----
    this->declare_parameter<std::string>("csv_path", "goals.csv");
    this->declare_parameter<std::string>("goal_frame", "map");

    csv_path_   = this->get_parameter("csv_path").as_string();
    goal_frame_ = this->get_parameter("goal_frame").as_string();

    // ---- 创建 action client ----
    action_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    // ---- 读取 CSV ----
    if (!load_goals_from_csv(csv_path_)) {
      RCLCPP_ERROR(this->get_logger(),
        "无法读取目标 CSV 文件: %s", csv_path_.c_str());
      init_ok_ = false;
      return;
    }
    RCLCPP_INFO(this->get_logger(),
      "已从 CSV 读取 %zu 个目标", goals_.size());

    start_timer_ = this->create_wall_timer(
      500ms, std::bind(&GoalPublisher::start_sending, this));
  }

  bool is_ok() const { return init_ok_; }

private:
  // -------------------- 读取 CSV --------------------
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
        if (!line.empty() && (std::isalpha(line[0]))) {
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
          "跳过格式错误的行: %s", line.c_str());
        continue;
      }
      goals_.push_back(g);
    }
    file.close();
    return !goals_.empty();
  }

  // -------------------- 启动:等 server 然后发第一个 --------------------
  void start_sending()
  {
    start_timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "等待 nav2 action server...");
    if (!action_client_->wait_for_action_server(20s)) {
      RCLCPP_ERROR(this->get_logger(),
        "20 秒内未发现 navigate_to_pose action server,退出");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Action server 已就绪");
    send_next_goal();
  }

  // -------------------- 发送下一个目标 --------------------
  void send_next_goal()
  {
    if (current_goal_index_ >= goals_.size()) {
      RCLCPP_INFO(this->get_logger(),
        "全部 %zu 个目标已处理完毕,节点关闭", goals_.size());
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
      "发送目标 %zu/%zu: x=%.2f y=%.2f theta=%.3f",
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

  // -------------------- 回调:目标被接受/拒绝 --------------------
  void goal_response_callback(const GoalHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_WARN(this->get_logger(),
        "目标 %zu 被服务器拒绝,跳到下一个",
        current_goal_index_ + 1);
      current_goal_index_++;
      send_next_goal();
    } else {
      RCLCPP_INFO(this->get_logger(), "目标已被接受,机器人开始移动");
    }
  }

  // -------------------- 回调:导航过程中的反馈 --------------------
  void feedback_callback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    static int counter = 0;
    if (counter++ % 20 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "  剩余距离: %.2f m", feedback->distance_remaining);
    }
  }

  // -------------------- 回调:目标结束(成功/中止/取消) --------------------
  void result_callback(const GoalHandle::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(),
          "目标 %zu 已到达", current_goal_index_ + 1);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(this->get_logger(),
          "目标 %zu 被中止 (aborted),继续下一个",
          current_goal_index_ + 1);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(),
          "目标 %zu 被取消 (canceled)", current_goal_index_ + 1);
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "未知的结果码");
        break;
    }

    current_goal_index_++;
    send_next_goal();
  }

  // -------------------- 成员变量 --------------------
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