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

    }

  private:
    void og_callback()
    {
      auto occupancy_grid_msg = nav_msgs::msg::OccupancyGrid();
      std::vector<signed char> og_array(25);
      for (int i=0;i<25;i++){
        og_array[i] = 0;  
      }

      occupancy_grid_msg.header.stamp = rclcpp::Clock().now();
      occupancy_grid_msg.header.frame_id = "map";

      occupancy_grid_msg.info.resolution = 1;

      occupancy_grid_msg.info.width = 5;
      occupancy_grid_msg.info.height = 5;

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

      
    }
      
   
    rclcpp::TimerBase::SharedPtr og_timer;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr og_publisher;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGrid_Publisher>());
  rclcpp::shutdown();
  return 0;
}