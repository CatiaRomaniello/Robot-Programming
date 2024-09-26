#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode()
        : Node("navigation_node")
    {
        initialpose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&NavigationNode::initialpose_callback, this, std::placeholders::_1));
        goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal", 10, std::bind(&NavigationNode::goal_callback, this, std::placeholders::_1));
    }

private:
    void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received /initialpose: [x: %f, y: %f, orientation: %f]",
                    msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.w);
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received /move_base_simple/goal: [x: %f, y: %f, orientation: %f]",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.orientation.w);
        }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationNode>());
    rclcpp::shutdown();
    return 0;
}


