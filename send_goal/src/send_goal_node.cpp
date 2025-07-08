#include <functional>
#include <memory>
#include <chrono>
#include <string>
#include <iostream>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>

using namespace std::chrono_literals;

class NavigateToPoseClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToPoseClient()
      : Node("navigate_to_pose_client")
  {
    this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // 等待action服务器，最多等10秒
    if (!this->client_ptr_->wait_for_action_server(10s))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Action server available. Ready to send goals.");

    // 你可以这里设置一个定时器，周期性发送目标，或者根据你的需求调用send_goal()
  }

  void send_goal(double x, double y, double yaw = 0.0)
  {
    if (!this->client_ptr_)
    {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();

    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->get_clock()->now();

    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;

    // 简单根据 yaw 角计算四元数
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = std::sin(yaw * 0.5);
    goal_msg.pose.pose.orientation.w = std::cos(yaw * 0.5);

    RCLCPP_INFO(this->get_logger(), "Sending goal to (%.2f, %.2f) with yaw %.2f radians", x, y, yaw);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NavigateToPoseClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NavigateToPoseClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&NavigateToPoseClient::result_callback, this, std::placeholders::_1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void cancel_goal()
  {
    if (goal_handle_)
    {
      RCLCPP_INFO(this->get_logger(), "Cancelling goal...");
      client_ptr_->async_cancel_goal(goal_handle_);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "No active goal to cancel");
    }
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  GoalHandleNavigateToPose::SharedPtr goal_handle_;

  void goal_response_callback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      goal_handle_ = goal_handle;
    }
  }

  void feedback_callback(
      GoalHandleNavigateToPose::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f meters", feedback->distance_remaining);
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult &result)
  {
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
    }

    // 在导航结束后，向txt追加一行 "done"
    std::string file_path = "/userdata/dev_ws/src/originbot/originbot_send_goal/goal_pose/goal_pose.txt"; // 确保这里定义了文件路径
    try
    {
      std::ofstream outfile;
      outfile.open(file_path, std::ios_base::app); // 以追加模式打开
      if (outfile.is_open())
      {
        outfile << "\ndone" << std::endl;
        outfile.close();
        RCLCPP_INFO(this->get_logger(), "写入 done 到文件");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "无法打开文件写入 done");
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "写入文件异常: %s", e.what());
    }

    // 可根据需求关闭节点或准备发送下一个目标
    rclcpp::shutdown();
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigateToPoseClient>();

  // 从文件中读取目标点
  double x = 0.0, y = 0.0, theta = 0.0;
  std::ifstream infile("/userdata/dev_ws/src/originbot/originbot_send_goal/goal_pose/goal_pose.txt");
  if (!infile)
  {
    std::cerr << "Failed to open goal.txt" << std::endl;
    return 1;
  }
  infile >> x >> y >> theta;

  // 传递给 send_goal
  node->send_goal(x, y, theta);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
