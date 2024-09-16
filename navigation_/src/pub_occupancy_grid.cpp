#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;


class OccupancyGrid_Publisher: public rclcpp::Node
{
  public:
    OccupancyGrid_Publisher()
    : Node("occupancy_grid_publisher") 
    {
      og_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("custom_occupancy_grid", 10);
      og_timer = this->create_wall_timer(500ms, std::bind(&OccupancyGrid_Publisher::og_callback,this));

      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

  private:
    void og_callback()
    {
      auto occupancy_grid_msg = nav_msgs::msg::OccupancyGrid();
      std::vector<signed char> og_array(35);
      for (int i=0;i<35;i++){
        og_array[i] = i % 2 == 0 ? 100: 0;  
      }

      occupancy_grid_msg.header.stamp = rclcpp::Clock().now();
      occupancy_grid_msg.header.frame_id = "map";

      occupancy_grid_msg.info.resolution = 1;

      occupancy_grid_msg.info.width = 5;
      occupancy_grid_msg.info.height = 7;

      occupancy_grid_msg.info.origin.position.x = 0.0;
      occupancy_grid_msg.info.origin.position.y = 0.0;
      occupancy_grid_msg.info.origin.position.z = 0.0;
      occupancy_grid_msg.info.origin.orientation.x = 0.0;
      occupancy_grid_msg.info.origin.orientation.y = 0.0;
      occupancy_grid_msg.info.origin.orientation.z = 0.0;
      occupancy_grid_msg.info.origin.orientation.w = 1.0;
      occupancy_grid_msg.data = og_array;


      
      //RCLCPP_INFO(this->get_logger(), "String Pub: '%s'", scan_msg.data.c_str());
      og_publisher->publish(occupancy_grid_msg);

      publish_tf();
    }
      void publish_tf()
    {
      auto transform_stamped = geometry_msgs::msg::TransformStamped();

      
      transform_stamped.header.stamp = this->get_clock()->now();
      transform_stamped.header.frame_id = "map";
      transform_stamped.child_frame_id = "odom";

      
      transform_stamped.transform.translation.x = 0.0;  
      transform_stamped.transform.translation.y = 0.0;
      transform_stamped.transform.translation.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, 0);  
      transform_stamped.transform.rotation.x = q.x();
      transform_stamped.transform.rotation.y = q.y();
      transform_stamped.transform.rotation.z = q.z();
      transform_stamped.transform.rotation.w = q.w();

      
      tf_broadcaster_->sendTransform(transform_stamped);
    }
    

   
    rclcpp::TimerBase::SharedPtr og_timer;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr og_publisher;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGrid_Publisher>());
  rclcpp::shutdown();
  return 0;
}