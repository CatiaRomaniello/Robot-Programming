#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>

class TransformBroadcasterNode : public rclcpp::Node {
public:
    TransformBroadcasterNode() : Node("dynamic_transform_broadcaster") {
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TransformBroadcasterNode::publish_base_to_laser, this));
    }

private:
    

    void publish_base_to_laser() {
        // Static transform between base_link and laser
        geometry_msgs::msg::TransformStamped base_to_laser;
        base_to_laser.header.stamp = this->get_clock()->now();
        base_to_laser.header.frame_id = "base_link";
        base_to_laser.child_frame_id = "Laser_frame";

        base_to_laser.transform.translation.x = 0.2;  // Example laser position
        base_to_laser.transform.translation.y = 0.0;
        base_to_laser.transform.translation.z = 0.1;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);  // Assuming no rotation
        base_to_laser.transform.rotation.x = q.x();
        base_to_laser.transform.rotation.y = q.y();
        base_to_laser.transform.rotation.z = q.z();
        base_to_laser.transform.rotation.w = q.w();

        broadcaster_->sendTransform(base_to_laser);
    }

    
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformBroadcasterNode>());
    rclcpp::shutdown();
    return 0;
}
