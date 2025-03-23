#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>
#include "project_messages/srv/find_wall.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

using findwall = project_messages::srv::FindWall;

class WallFollower : public rclcpp::Node
{
public:
  WallFollower() : Node("simple_subscriber")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&WallFollower::timer_callback, this));
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                                    "scan", 10, std::bind(&WallFollower::sensor_callback, this, _1));
  }

private:
  void timer_callback()
  {
    float rot = 0.5;
    float fwd_turn = 0.05;
    if(laser_forward < 0.5){
        twist_msg.angular.z = rot*6;
        twist_msg.linear.x = fwd_turn;
    }
    if(laser_right <= 0.3 && laser_right >= 0.2){
        twist_msg.linear.x = fwd_turn;
        twist_msg.angular.z = 0.0;
    }
    else if(laser_right > 0.3 && laser_forward >= 0.5){
        twist_msg.angular.z = -0.2;
        twist_msg.linear.x = fwd_turn;
    }
    else if(laser_right < 0.2){
        twist_msg.angular.z = rot+0.2;
        twist_msg.linear.x = fwd_turn;
    }

    publisher_->publish(this->twist_msg);
    }

    void sensor_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        this->laser_left = msg->ranges[90];
        this->laser_forward = msg->ranges[0];
        this->laser_right = msg->ranges[270];

        //RCLCPP_INFO(this->get_logger(), "[LEFT] = '%f'", this->laser_left);
        //RCLCPP_INFO(this->get_logger(), "[RIGHT] = '%f'", this->laser_right);
        //RCLCPP_INFO(this->get_logger(), "[FORWARD] = '%f'", this->laser_forward);
    }

  geometry_msgs::msg::Twist twist_msg;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  float laser_left;
  float laser_right;
  float laser_forward;
  float laser_back;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_client");
  rclcpp::Client<findwall>::SharedPtr client = node->create_client<findwall>("find_wall");

  auto request = std::make_shared<findwall::Request>();

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result_future = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot is moving");
  } 
  else 
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /moving");
  }

  rclcpp::spin(std::make_shared<WallFollower>());

  rclcpp::shutdown();
  return 0;
}