#include <inttypes.h>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "project_messages/action/odom_record.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/detail/float64__struct.hpp"
#include "std_msgs/msg/float64.hpp"


class MyActionClient : public rclcpp::Node {
public:
  using OdomRecord = project_messages::action::OdomRecord;
  using GoalHandleOdomRecord = rclcpp_action::ClientGoalHandle<OdomRecord>;


  explicit MyActionClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("my_action_client", node_options), goal_done_(false) {
    this->client_ptr_ = rclcpp_action::create_client<OdomRecord>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "record_odom");

    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&MyActionClient::send_goal, this));
  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available");
        return;
    }

    auto goal_msg = OdomRecord::Goal(); // Create an empty goal message

    auto send_goal_options = rclcpp_action::Client<OdomRecord>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&MyActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&MyActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&MyActionClient::result_callback, this, std::placeholders::_1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}


private:
  rclcpp_action::Client<OdomRecord>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  void goal_response_callback(std::shared_future<GoalHandleOdomRecord::SharedPtr> future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

  void feedback_callback(GoalHandleOdomRecord::SharedPtr,
                                        const std::shared_ptr<const OdomRecord::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Feedback received: %f", feedback->current_total);
    }

  void result_callback(const GoalHandleOdomRecord::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
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
    RCLCPP_INFO(this->get_logger(), "Total distance: %f", result.result->list_of_odoms);
    }

}; // class MyActionClient

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<MyActionClient>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_client);

  while (!action_client->is_goal_done()) {
    executor.spin_some();
  }

  rclcpp::shutdown();
  return 0;
}