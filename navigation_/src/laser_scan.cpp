#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;


class LaserPublisher: public rclcpp::Node
{
  public:
    LaserPublisher()
    : Node("minimal_publisher") 
    {
      laser_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("Scan", 10);
      laser_timer = this->create_wall_timer(500ms, std::bind(&LaserPublisher::laser_callback,this));

      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this); 
    }

  private:
    void laser_callback()
    {
      std::vector<float> range(100,10.0f); 


      auto scan_msg = sensor_msgs::msg::LaserScan();
      scan_msg.header.stamp= this->now(); 
      scan_msg.header.frame_id = "Laser_frame";
      scan_msg.angle_min= 0;
      scan_msg.angle_max= 3.14;
      scan_msg.angle_increment = 0.1;
      scan_msg.range_min = 0.0;
      scan_msg.range_max = 10.0;
      scan_msg.ranges = range;
      
      auto transform_stamped = geometry_msgs::msg::TransformStamped();
      transform_stamped.header.stamp = this->get_clock()->now();
      transform_stamped.header.frame_id = "odom";

      transform_stamped.child_frame_id = "base_link";

      transform_stamped.transform.translation.x = 0.2;
      transform_stamped.transform.translation.y = 0.0;
      transform_stamped.transform.translation.z = 0.3;

      transform_stamped.transform.rotation.x = 0.0;
      transform_stamped.transform.rotation.y = 0.0;
      transform_stamped.transform.rotation.z = 0.0;
      transform_stamped.transform.rotation.w = 1.0;

      tf_broadcaster_->sendTransform(transform_stamped);
      laser_publisher->publish(scan_msg);

      
    }

   
    rclcpp::TimerBase::SharedPtr laser_timer;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserPublisher>());
  rclcpp::shutdown();
  return 0;
}