#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>

class TransformBroadcasterNode : public rclcpp::Node {
public:
    TransformBroadcasterNode() : Node("dynamic_transform_broadcaster") {
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        base_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/base_pub", 10, std::bind(&TransformBroadcasterNode::base_callback, this, std::placeholders::_1));


        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TransformBroadcasterNode::publish_map_to_odom, this));

        publish_base_to_laser();
    }

private:
    
    void base_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        (void)msg;
        geometry_msgs::msg::TransformStamped base_to_laser;
        base_to_laser.header.stamp = this->get_clock()->now();
        base_to_laser.header.frame_id = "base_link";
        base_to_laser.child_frame_id = "Laser_frame";

        // Usa le informazioni di odometria per aggiornare la trasformazione
        base_to_laser.transform.translation.x = 0.2;  // o un valore derivato dai dati di odometria
        base_to_laser.transform.translation.y = 0.0;
        base_to_laser.transform.translation.z = 0.1;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);  // Assumendo nessuna rotazione tra base_link e laser
        base_to_laser.transform.rotation.x = q.x();
        base_to_laser.transform.rotation.y = q.y();
        base_to_laser.transform.rotation.z = q.z();
        base_to_laser.transform.rotation.w = q.w();

        broadcaster_->sendTransform(base_to_laser); 
    }

    void publish_map_to_odom() {
        // Static transform between map and odom
        geometry_msgs::msg::TransformStamped map_to_odom;
        map_to_odom.header.stamp = this->get_clock()->now();
        map_to_odom.header.frame_id = "map";
        map_to_odom.child_frame_id = "odom";

        map_to_odom.transform.translation.x = 0.0;
        map_to_odom.transform.translation.y = 0.0;
        map_to_odom.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);  // No rotation between map and odom
        map_to_odom.transform.rotation.x = q.x();
        map_to_odom.transform.rotation.y = q.y();
        map_to_odom.transform.rotation.z = q.z();
        map_to_odom.transform.rotation.w = q.w();

        broadcaster_->sendTransform(map_to_odom);
    }

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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr base_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformBroadcasterNode>());
    rclcpp::shutdown();
    return 0;
}
