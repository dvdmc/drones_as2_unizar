#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using std::placeholders::_1;

class VisionPoseRepublisher : public rclcpp::Node
{
public:
    VisionPoseRepublisher()
        : Node("vision_pose_republisher")
    {
        // Declare default parameters (relative topic names)
        this->declare_parameter<std::string>("pose_in_topic", "self_localization/pose");
        this->declare_parameter<std::string>("twist_in_topic", "self_localization/twist");
        this->declare_parameter<std::string>("pose_out_topic", "mavros/vision_pose/pose");
        this->declare_parameter<std::string>("twist_out_topic", "mavros/vision_speed/speed_twist");

        std::string pose_in = this->get_parameter("pose_in_topic").as_string();
        std::string twist_in = this->get_parameter("twist_in_topic").as_string();
        std::string pose_out = this->get_parameter("pose_out_topic").as_string();
        std::string twist_out = this->get_parameter("twist_out_topic").as_string();

        // Input QoS (matches upstream publisher)
        rclcpp::QoS in_qos(10);
        in_qos.best_effort();  // or whatever the localization publisher uses

        // Output QoS (matches downstream subscriber, MAVROS)
        rclcpp::QoS out_qos(10);
        out_qos.reliable();

        // Subscribers
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_in, in_qos, std::bind(&VisionPoseRepublisher::pose_callback, this, _1));
        twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            twist_in, in_qos, std::bind(&VisionPoseRepublisher::twist_callback, this, _1));

        // Publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_out, out_qos);
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(twist_out, out_qos);

        RCLCPP_INFO(this->get_logger(), "Subscribed (input QoS): %s / %s", pose_in.c_str(), twist_in.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing (output QoS): %s / %s", pose_out.c_str(), twist_out.c_str());
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pose_pub_->publish(*msg);
    }

    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        twist_pub_->publish(*msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionPoseRepublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
