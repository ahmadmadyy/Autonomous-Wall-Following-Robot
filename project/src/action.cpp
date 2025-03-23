#include <functional>
#include <math.h>
#include <memory>
#include <thread>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "project_messages/action/odom_record.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/detail/float64__struct.hpp"
#include "std_msgs/msg/float64.hpp"

class MyActionServer : public rclcpp::Node
{
public:
  using OdomRecord = project_messages::action::OdomRecord;
  using GoalHandleOdomRecord = rclcpp_action::ServerGoalHandle<OdomRecord>;

  explicit MyActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("my_action_server", options)
  {
    using namespace std::placeholders;

    subscriber_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    action_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions options_subs;
    options_subs.callback_group = subscriber_group_;

    this->action_server_ = rclcpp_action::create_server<OdomRecord>(this, "record_odom",
            std::bind(&MyActionServer::handle_goal, this, _1, _2),
            std::bind(&MyActionServer::handle_cancel, this, _1),
            std::bind(&MyActionServer::handle_accepted, this, _1), rcl_action_server_get_default_options(), action_group_);

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                        "odom", 10, std::bind(&MyActionServer::topic_callback, this, _1), options_subs);

    //publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    //publisher2_ = this->create_publisher<std_msgs::msg::Float64>("total_distance", 10);

    distance_ = 0;
  }

private:
  rclcpp_action::Server<OdomRecord>::SharedPtr action_server_;
  //rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher2_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::CallbackGroup::SharedPtr subscriber_group_;
  rclcpp::CallbackGroup::SharedPtr action_group_;
  double distance_;
  nav_msgs::msg::Odometry prev_pose_;
  geometry_msgs::msg::Point32 pose_;
  geometry_msgs::msg::Point32 init_pose_;
  //geometry_msgs::msg::Point32[] odoms_ = {};
  std::vector<geometry_msgs::msg::Point32> odoms_; 
  bool first_time_ = false;


  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double dx = msg->pose.pose.position.x - prev_pose_.pose.pose.position.x;
    double dy = msg->pose.pose.position.y - prev_pose_.pose.pose.position.y;
    double dz = msg->pose.pose.position.z - prev_pose_.pose.pose.position.z;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);
    this->distance_ += distance;
    prev_pose_.pose = msg->pose;

    this->pose_.x = msg->pose.pose.position.x;
    this->pose_.y = msg->pose.pose.position.y;
    this->pose_.z = msg->pose.pose.orientation.z;; // Store theta as z

  }


  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const OdomRecord::Goal> goal)
  {
    //RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d", goal->seconds);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleOdomRecord> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleOdomRecord> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleOdomRecord> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<OdomRecord::Feedback>();
    auto &tracking_distance_FB = feedback->current_total;
    tracking_distance_FB = this->distance_;
    auto result = std::make_shared<OdomRecord::Result>();
    rclcpp::Rate loop_rate(1); // 1 Hz

    // Initialize start position
    this->init_pose_ = this->pose_;
    this->odoms_.clear(); // Clear previous odometry data

    /*do {
        // Check for cancel requests
        if (goal_handle->is_canceling()) {
            result->list_of_odoms = this->odoms_;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        // Record odometry and provide feedback
        this->odoms_.push_back(this->pose_);
        tracking_distance_FB = distance_; // Update feedback with the total distance
        goal_handle->publish_feedback(feedback);

        // Debug logs
        RCLCPP_INFO(this->get_logger(), "Publishing feedback: Total Distance = %f", tracking_distance_FB);
        RCLCPP_INFO(this->get_logger(), "Current Pose: X=%f, Y=%f", this->pose_.x, this->pose_.y);

        loop_rate.sleep();
    } while ((sqrt(pow(this->pose_.x - this->init_pose_.x, 2) +
               pow(this->pose_.y - this->init_pose_.y, 2)) > 0.2) &&
         rclcpp::ok());*/

    do {
        double dx = this->pose_.x - this->init_pose_.x;
        double dy = this->pose_.y - this->init_pose_.y;
        double distance_to_start = sqrt(dx * dx + dy * dy);

        if (distance_to_start < 0.2 && this->odoms_.size() > 10) {  // Ensures at least 10 odom readings
            RCLCPP_INFO(this->get_logger(), "Full lap detected. Stopping recording.");
            break;
        }

        loop_rate.sleep();
    } while (rclcpp::ok());


    // Final result
    result->list_of_odoms = this->odoms_;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }

};  // class MyActionServer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<MyActionServer>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}