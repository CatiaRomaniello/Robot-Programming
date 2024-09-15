#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp" 
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;



class MobileBasePublisher : public rclcpp::Node
{
  public:
    MobileBasePublisher()
    : Node("mobile_base_publisher"), x_(0.0), y_(0.0), theta_(0.0), v_(1.0), w_(0.1)
    {
      base_publisher = this->create_publisher<nav_msgs::msg::Odometry>("base_pub", 10);
      base_timer= this->create_wall_timer(500ms, std::bind(&MobileBasePublisher::base_callback,this));

      
    }

  private:
    void base_callback()
    {

      double dt = 0.1;  
      x_ += v_ * std::cos(theta_) * dt;
      y_ += v_ * std::sin(theta_) * dt;
      theta_ += w_ * dt;


      auto odom_msg = nav_msgs::msg::Odometry();
      odom_msg.header.stamp = this->get_clock()->now();
      odom_msg.header.frame_id = "map_frame1";

      odom_msg.pose.pose.position.x = x_;
      odom_msg.pose.pose.position.y = y_;
      odom_msg.pose.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, theta_);
      odom_msg.pose.pose.orientation.x = q.x();
      odom_msg.pose.pose.orientation.y = q.y();
      odom_msg.pose.pose.orientation.z = q.z();
      odom_msg.pose.pose.orientation.w = q.w();

      odom_msg.twist.twist.linear.x = v_;
      odom_msg.twist.twist.angular.z = w_;

      base_publisher->publish(odom_msg);

      RCLCPP_INFO(this->get_logger(), "Odom: x=%.2f, y=%.2f, theta=%.2f", x_, y_, theta_);
    }
    
    double x_, y_, theta_, v_, w_;
    rclcpp::TimerBase::SharedPtr base_timer;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr base_publisher;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MobileBasePublisher>());
  rclcpp::shutdown();
  return 0;
}