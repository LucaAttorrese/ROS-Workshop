#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class TfSubPub : public rclcpp::Node {
public:
    TfSubPub() : Node("odom_to_tf") {
        this->declare_parameter<std::string>("root_frame", "world");
        this->declare_parameter<std::string>("child_frame", "wheel_odom");

        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "input_odom", 10,
            std::bind(&TfSubPub::callback, this, std::placeholders::_1)
        );

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::string root_frame = this->get_parameter("root_frame").as_string();
        std::string child_frame = this->get_parameter("child_frame").as_string();

        if (msg->header.frame_id == "odom") {
            root_frame = "world";
            child_frame = "wheel_odom";
        } else {
            root_frame = "world";
            child_frame = "gps_odom";
        }

        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = root_frame;
        transformStamped.child_frame_id = child_frame;

        transformStamped.transform.translation.x = msg->pose.pose.position.x;
        transformStamped.transform.translation.y = msg->pose.pose.position.y;
        transformStamped.transform.translation.z = msg->pose.pose.position.z;

        transformStamped.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(transformStamped);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfSubPub>());
    rclcpp::shutdown();
    return 0;
}
