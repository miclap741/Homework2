#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "ros2_kdl_package/action/ros2_kdl_traj.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"


class KdlActionClient : public rclcpp::Node
{
public:
  using Ros2KdlTraj = ros2_kdl_package::action::Ros2KdlTraj;
  using GoalHandleRos2KdlTraj = rclcpp_action::ClientGoalHandle<Ros2KdlTraj>;


  KdlActionClient() : KdlActionClient(rclcpp::NodeOptions()) {} //aggiunto adesso

  explicit KdlActionClient(const rclcpp::NodeOptions & options)
  : Node("kdl_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Ros2KdlTraj>(
      this,
      "ros2_kdl_traj");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&KdlActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Ros2KdlTraj::Goal();
    goal_msg.traj_duration= 1.5;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Ros2KdlTraj>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&KdlActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&KdlActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&KdlActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Ros2KdlTraj>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleRos2KdlTraj::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleRos2KdlTraj::SharedPtr,
    const std::shared_ptr<const Ros2KdlTraj::Feedback> feedback)
  {
   
    RCLCPP_INFO(this->get_logger(), "%f", feedback->error);
  }

  void result_callback(const GoalHandleRos2KdlTraj::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "%f",result.result->final_error);
    rclcpp::shutdown();
  }
};  // class KdlActionClient


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KdlActionClient>());
    rclcpp::shutdown();
    return 0;
} // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(KdlActionClient)
