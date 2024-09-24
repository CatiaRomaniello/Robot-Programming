#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class OdometryPublisher : public rclcpp::Node {
public:
  OdometryPublisher()
  : Node("odometry_publisher"), x(0.0), y(0.0), th(0.0), right_wheel_est_vel(0.2), left_wheel_est_vel(0.1), wheel_separation(0.5)
  {
    // Publisher for odometry messages
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    
    // Broadcaster for the TF
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    // Set the initial time
    last_time = this->get_clock()->now();
    
    // Timer to call the update function periodically (1 Hz)
    timer_ = this->create_wall_timer(1s, std::bind(&OdometryPublisher::update_odometry, this));
  }

private:
  void update_odometry()
  {
    auto current_time = this->get_clock()->now();
    
    // Compute the time difference (dt)
    double dt = (current_time - last_time).seconds();

    // Calculate linear and angular velocities based on wheel velocities
    double linear = (right_wheel_est_vel + left_wheel_est_vel) / 2.0;
    double angular = (right_wheel_est_vel - left_wheel_est_vel) / wheel_separation;

    // Compute change in position and orientation
    double delta_x = linear * cos(th) * dt;
    double delta_y = linear * sin(th) * dt;
    double delta_th = angular * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    // Create quaternion from yaw (th)
    tf2::Quaternion quat;
    quat.setRPY(0, 0, th);

    // Prepare the transform message
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation.x = quat.x();
    odom_trans.transform.rotation.y = quat.y();
    odom_trans.transform.rotation.z = quat.z();
    odom_trans.transform.rotation.w = quat.w();

    // Send the transform
    tf_broadcaster_->sendTransform(odom_trans);

    // Prepare the odometry message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // Set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();

    // Set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = linear;     // Updated linear velocity
    odom.twist.twist.angular.z = angular;   // Updated angular velocity

    // Publish the odometry message
    odom_pub->publish(odom);

    // Update the last time
    last_time = current_time;
  }

  // Variables for pose and velocity
  double x, y, th;
  double right_wheel_est_vel, left_wheel_est_vel, wheel_separation; // Wheel velocities and separation

  // ROS 2 Publishers and Broadcasters
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Timer for the update function
  rclcpp::TimerBase::SharedPtr timer_;

  // Time tracking
  rclcpp::Time last_time;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<OdometryPublisher>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
