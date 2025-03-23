#include <rclcpp/rclcpp.hpp>
#include "rclcpp/timer.hpp"
#include "project_messages/srv/find_wall.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>
#include <vector>
#include <chrono>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;

using findwall = project_messages::srv::FindWall;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node
{
public:
  ServerNode() : Node("findwall_server")
  {
    callback_group1_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    callback_group2_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions options2;
    options2.callback_group = callback_group2_;

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    srv_ = this->create_service<findwall>(
         "find_wall", std::bind(&ServerNode::findwall_callback, this, _1, _2), 
            rmw_qos_profile_services_default, callback_group1_);

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ServerNode::sensor_callback, this, _1), options2);

  }

private:
  geometry_msgs::msg::Twist twist_msg;
  rclcpp::CallbackGroup::SharedPtr callback_group1_;
  rclcpp::CallbackGroup::SharedPtr callback_group2_;
  //rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<findwall>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  float laser_left;
  float laser_right;
  float laser_forward;
  float laser_back;
  float shortest;

  void findwall_callback(const std::shared_ptr<findwall::Request> request, 
                                                const std::shared_ptr<findwall::Response> response) 
  {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0;
    message.angular.z = 0.1;

    // Find shortest and turn to it
    shortest = findMin(laser_left, laser_right, laser_forward, laser_back);
    while(abs(this->laser_forward - shortest) >= 0.05)
    {
        publisher_->publish(message);
        //RCLCPP_INFO(this->get_logger(), "[FORWARD S] = '%f'", this->laser_forward);
        //RCLCPP_INFO(this->get_logger(), "[MIN DIST] = '%f'", shortest);
        shortest = findMin(laser_left, laser_right, laser_forward, laser_back);
    }
    std::this_thread::sleep_for(100ms);
    message.angular.z = 0;
    publisher_->publish(message);

    // Move forward
    while(abs(this->laser_forward) >= 0.3)
    {
        message.linear.x = 0.05;
        message.angular.z = 0;
        publisher_->publish(message);
        //RCLCPP_INFO(this->get_logger(), "[FORWARD S] = '%f'", this->laser_forward);
    }
    std::this_thread::sleep_for(100ms);
    message.linear.x = 0;
    message.angular.z = 0;
    publisher_->publish(message);

     // Find shortest and turn to it
    message.linear.x = 0;
    message.angular.z = 0.5;
    shortest = findMin(laser_left, laser_right, laser_forward, laser_back);
    while(100*abs(this->laser_right - shortest) >= 0.005 && shortest >= 0.1) // Simple P Control
    {
        publisher_->publish(message);
        //RCLCPP_INFO(this->get_logger(), "[RIGHT] = '%f'", this->laser_right);
        //RCLCPP_INFO(this->get_logger(), "[MIN DIST] = '%f'", shortest);
        shortest = findMin(laser_left, laser_right, laser_forward, laser_back);
    }
    std::this_thread::sleep_for(100ms);
    message.angular.z = 0;
    publisher_->publish(message);
    response->wallfound = true;
  }

  float findMin(const float &w, const float &x, const float &y, const float &z)
  {
    std::vector<float> mins = {w,x,y,z};
    float min = w;
    for(auto it = mins.begin(); it != mins.end(); ++it)
    {
        if(*it < min)
        {
            min = *it;
        }
    }
    return min;
  }

  void sensor_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    this->laser_left = msg->ranges[90];
    this->laser_forward = msg->ranges[0];
    this->laser_right = msg->ranges[270];
    this->laser_back = msg->ranges[180];

    //RCLCPP_INFO(this->get_logger(), "[LEFT] = '%f'", this->laser_left);
    //RCLCPP_INFO(this->get_logger(), "[RIGHT] = '%f'", this->laser_right);
    //RCLCPP_INFO(this->get_logger(), "[FORWARD] = '%f'", this->laser_forward);
    //RCLCPP_INFO(this->get_logger(), "[BACK] = '%f'", this->laser_back);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //rclcpp::spin(std::make_shared<ServerNode>());
  std::shared_ptr<ServerNode> find_wall_node = std::make_shared<ServerNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(find_wall_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}