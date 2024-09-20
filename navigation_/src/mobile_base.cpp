#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp" 
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "visualization_msgs/msg/marker.hpp"


using namespace std::chrono_literals;



class MobileBasePublisher : public rclcpp::Node
{
  public:
    MobileBasePublisher()
    : Node("mobile_base_publisher"), x_(0.0), y_(0.0), theta_(0.0), v_(0.0), w_(0.0)
    {
      base_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
      base_timer= this->create_wall_timer(500ms, std::bind(&MobileBasePublisher::base_callback,this));
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this); 
      marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

      cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MobileBasePublisher::cmd_vel_callback, this, std::placeholders::_1));
      
    }

  private:

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
      {
        v_ = msg->linear.x; 
        w_ = msg->angular.z; 
    }
    void base_callback()
    {

      double dt = 0.1;  
      x_ += v_ * std::cos(theta_) * dt;
      y_ += v_ * std::sin(theta_) * dt;
      theta_ += w_ * dt;


      auto odom_msg = nav_msgs::msg::Odometry();
      odom_msg.header.stamp = this->get_clock()->now();
      odom_msg.header.frame_id = "odom";

      odom_msg.child_frame_id = "base_link";

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

      auto transform_stamped = geometry_msgs::msg::TransformStamped();
      transform_stamped.header.stamp = this->get_clock()->now();
      transform_stamped.header.frame_id = "odom";

      transform_stamped.child_frame_id = "base_link";

      transform_stamped.transform.translation.x = x_;
      transform_stamped.transform.translation.y = y_;
      transform_stamped.transform.translation.z = 0.0;
      transform_stamped.transform.rotation.x = q.x();
      transform_stamped.transform.rotation.y = q.y();
      transform_stamped.transform.rotation.z = q.z();
      transform_stamped.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(transform_stamped);

      RCLCPP_INFO(this->get_logger(), "Odom: x=%.2f, y=%.2f, theta=%.2f", x_, y_, theta_);
      publish_marker();
      publish_map_to_odom();
      
    }
    
    void publish_marker()
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";  
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "robot_shape";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;  
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;  
    marker.scale.y = 0.5;
    marker.scale.z = 0.1; 

    marker.color.a = 1.0;  // Trasparenza
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;  // Verde

    marker_publisher->publish(marker);
}
void publish_map_to_odom() {
      geometry_msgs::msg::TransformStamped map_to_odom;
      map_to_odom.header.stamp = this->get_clock()->now();
      map_to_odom.header.frame_id = "map";
      map_to_odom.child_frame_id = "odom";

      map_to_odom.transform.translation.x = 0.0;
      map_to_odom.transform.translation.y = 0.0;
      map_to_odom.transform.translation.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, 0);  
      map_to_odom.transform.rotation.x = q.x();
      map_to_odom.transform.rotation.y = q.y();
      map_to_odom.transform.rotation.z = q.z();
      map_to_odom.transform.rotation.w = q.w();

      
      tf_broadcaster_->sendTransform(map_to_odom);
    }
    double x_, y_, theta_, v_, w_;
    rclcpp::TimerBase::SharedPtr base_timer;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr base_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MobileBasePublisher>());
  rclcpp::shutdown();
  return 0;
}
