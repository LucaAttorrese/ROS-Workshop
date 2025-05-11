#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class odom_to_tf : public rclcpp::Node {
public:
    odom_to_tf() : Node("odom_to_tf") {
        this->declare_parameter<std::string>("root_frame", "world");

        // Subscribe to a generic topic "Input_odom". This name can be remapped to reuse this node in different projects
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "input_odom", 10,
            std::bind(&odom_to_tf::callback, this, std::placeholders::_1)
        );

        // Create the tf publisher
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::string root_frame = this->get_parameter("root_frame").as_string();
        std::string child_frame;
        double offset_x = 0.0;
        double offset_y = 0.0;
        double offset_yaw = 0.0;


        // If clause to manage different types of odometry messages
        if (msg->header.frame_id == "odom") {
            // Prevously calculated offset between wheel and gps frame
            child_frame = "wheel_odom";
            offset_x = 0.01393;
            offset_y = -2.282713;
            offset_yaw = 3.1416; // 180°
        } else {
            child_frame = "gps_odom";
        }

        // Tf2 initialization
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = msg->header.stamp;
        transformStamped.header.frame_id = root_frame;
        transformStamped.child_frame_id = child_frame;

        // Translation with offset
        transformStamped.transform.translation.x = msg->pose.pose.position.x + offset_x;
        transformStamped.transform.translation.y = msg->pose.pose.position.y + offset_y;
        transformStamped.transform.translation.z = msg->pose.pose.position.z;

        // Orientation with offset (used only by wheel_odom)
        tf2::Quaternion q_orig, q_rot, q_final;
        q_orig.setX(msg->pose.pose.orientation.x);
        q_orig.setY(msg->pose.pose.orientation.y);
        q_orig.setZ(msg->pose.pose.orientation.z);
        q_orig.setW(msg->pose.pose.orientation.w);

        q_rot.setRPY(0, 0, offset_yaw);  // only yaw
        q_final = q_rot * q_orig;
        q_final.normalize();

        if (child_frame == "wheel_odom") {
            transformStamped.transform.rotation.x = q_final.x();
            transformStamped.transform.rotation.y = q_final.y();
            transformStamped.transform.rotation.z = q_final.z();
            transformStamped.transform.rotation.w = q_final.w();
        } else {
            transformStamped.transform.rotation = msg->pose.pose.orientation;
        }

        tf_broadcaster_->sendTransform(transformStamped);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<odom_to_tf>());
    rclcpp::shutdown();
    
    return 0;
}
