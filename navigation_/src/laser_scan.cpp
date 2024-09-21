#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class LaserPublisher : public rclcpp::Node
{
public:
    LaserPublisher()
        : Node("laser_publisher")
    {
        laser_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("Scan", 100);
        laser_timer_ = this->create_wall_timer(500ms, std::bind(&LaserPublisher::laser_callback, this));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_timer_ = this->create_wall_timer(500ms, std::bind(&LaserPublisher::publish_tf, this));
    }

private:
    void laser_callback()
    {
        
        std::vector<float> range(100);
        for (int i = 0; i < 100; ++i)
        {
            range[i] = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 10.0));
        }

        auto scan_msg = sensor_msgs::msg::LaserScan();
        scan_msg.header.stamp = this->now();
        scan_msg.header.frame_id = "Laser_frame"; 

        scan_msg.angle_min= -1.57;
        scan_msg.angle_max= 1.57;
        scan_msg.angle_increment = 0.0314;
        scan_msg.range_min = 0.0;
        scan_msg.range_max = 10.0;
        scan_msg.ranges = range; 

        laser_publisher_->publish(scan_msg);
    }

    void publish_tf()
    {
        
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "base_link";  
        transform_stamped.child_frame_id = "Laser_frame"; 

        transform_stamped.transform.translation.x = 0.2; 
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.1;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0); 
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform_stamped);
    }

    rclcpp::TimerBase::SharedPtr laser_timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher_;

    rclcpp::TimerBase::SharedPtr tf_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserPublisher>());
    rclcpp::shutdown();
    return 0;
}
