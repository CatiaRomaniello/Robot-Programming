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
      broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

  private:
    void og_callback()
    {
      auto occupancy_grid_msg = nav_msgs::msg::OccupancyGrid();
      std::vector<signed char> og_array(100);
      int maze[10][10] = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {100, 0, 0, 0, 100, 0, 0, 0, 0, 100},
        {0, 0, 100, 0, 100, 0, 100, 100, 0, 100},
        {100, 0, 100, 0, 0, 0, 0, 100, 0, 100},
        {100, 0, 100, 100, 100, 100, 0, 100, 0, 100},
        {0, 0, 0, 0, 0, 100, 0, 100, 0, 100},
        {100, 100, 100, 100, 0, 100, 0, 100, 0, 100},
        {100, 0, 0, 100, 0, 100, 0, 100, 0, 100},
        {100, 0, 0, 0, 0, 0, 0, 0, 0, 100},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
      };

      for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
          og_array[i * 10 + j] = maze[i][j];
        }
      }

      occupancy_grid_msg.header.stamp = this->get_clock()->now();
      occupancy_grid_msg.header.frame_id = "map";

      occupancy_grid_msg.info.resolution = 1;

      occupancy_grid_msg.info.width = 10;
      occupancy_grid_msg.info.height = 10;

      occupancy_grid_msg.info.origin.position.x = -5.0;
      occupancy_grid_msg.info.origin.position.y = -5.0;
      occupancy_grid_msg.info.origin.position.z = 0.0;
      occupancy_grid_msg.info.origin.orientation.x = 0.0;
      occupancy_grid_msg.info.origin.orientation.y = 0.0;
      occupancy_grid_msg.info.origin.orientation.z = 0.0;
      occupancy_grid_msg.info.origin.orientation.w = 1.0;
      occupancy_grid_msg.data = og_array;

      og_publisher->publish(occupancy_grid_msg);

      publish_map_to_odom();
      
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
      q.setRPY(0, 0, 0);  // Nessuna rotazione
      map_to_odom.transform.rotation.x = q.x();
      map_to_odom.transform.rotation.y = q.y();
      map_to_odom.transform.rotation.z = q.z();
      map_to_odom.transform.rotation.w = q.w();

      // Invia la trasformazione
      broadcaster_->sendTransform(map_to_odom);
    }
   
    rclcpp::TimerBase::SharedPtr og_timer;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr og_publisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGrid_Publisher>());
  rclcpp::shutdown();
  return 0;
}