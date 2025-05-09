#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#define a 6378137.0
#define b 6356752.0

class gps_to_odom : public rclcpp::Node {
public:
    gps_to_odom() : Node("gps_to_odom"), lat1_(0.0), lon1_(0.0) {
        e_squared_ = 1.0 - ((b * b) / (a * a));

        this->declare_parameter("lat_r", 0.0);
        this->declare_parameter("lon_r", 0.0);
        this->declare_parameter("alt_r", 0.0);

        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "fix", 10,
            std::bind(&GpsToOdom::gpsCallback, this, std::placeholders::_1)
        );

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("gps_odom", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    double lat1_, lon1_;
    double e_squared_;

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        double lat_r, lon_r, alt_r;
        this->get_parameter("lat_r", lat_r);
        this->get_parameter("lon_r", lon_r);
        this->get_parameter("alt_r", alt_r);

        double lat_rad = msg->latitude * M_PI / 180.0;
        double lon_rad = msg->longitude * M_PI / 180.0;
        double alt = msg->altitude;

        double sq = sqrt(1 - e_squared_ * pow(sin(lat_rad), 2));
        double N = a / sq;

        double x_ecef = (N + alt) * cos(lat_rad) * cos(lon_rad);
        double y_ecef = (N + alt) * cos(lat_rad) * sin(lon_rad);
        double z_ecef = (N * (1 - e_squared_) + alt) * sin(lat_rad);

        // Reference point ECEF
        double lat_r_rad = lat_r * M_PI / 180.0;
        double lon_r_rad = lon_r * M_PI / 180.0;

        double sq_r = sqrt(1 - e_squared_ * pow(sin(lat_r_rad), 2));
        double N_r = a / sq_r;

        double x_r = (N_r + alt_r) * cos(lat_r_rad) * cos(lon_r_rad);
        double y_r = (N_r + alt_r) * cos(lat_r_rad) * sin(lon_r_rad);
        double z_r = (N_r * (1 - e_squared_) + alt_r) * sin(lat_r_rad);

        // ECEF to ENU
        double x_enu = -sin(lon_r_rad) * (x_ecef - x_r) + cos(lon_r_rad) * (y_ecef - y_r);
        double y_enu = -sin(lat_r_rad) * cos(lon_r_rad) * (x_ecef - x_r)
                     - sin(lat_r_rad) * sin(lon_r_rad) * (y_ecef - y_r)
                     + cos(lat_r_rad) * (z_ecef - z_r);
        double z_enu = cos(lat_r_rad) * cos(lon_r_rad) * (x_ecef - x_r)
                     + cos(lat_r_rad) * sin(lon_r_rad) * (y_ecef - y_r)
                     + sin(lat_r_rad) * (z_ecef - z_r);

        // Orientamento
        double lat2 = y_enu;
        double lon2 = x_enu;

        double theta = 0.0;
        double quat_x = 0.0, quat_y = 0.0, quat_z = 0.0, quat_w = 1.0;

        if (lon2 != 0.0) {
            double lat1_rad = lat1_ * M_PI / 180.0;
            double lon1_rad = lon1_ * M_PI / 180.0;
            double lat2_rad = lat2 * M_PI / 180.0;
            double lon2_rad = lon2 * M_PI / 180.0;

            double Y = sin(lon2_rad - lon1_rad) * cos(lat2_rad);
            double X = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(lon2_rad - lon1_rad);
            theta = atan2(Y, X);

            lon1_ = lon2;
            lat1_ = lat2;

            quat_z = sin(theta / 2);
            quat_w = cos(theta / 2);
        }

        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "world";
        odom_msg.child_frame_id = "gps_odom";

        odom_msg.pose.pose.position.x = x_enu;
        odom_msg.pose.pose.position.y = y_enu;
        odom_msg.pose.pose.position.z = z_enu;

        odom_msg.pose.pose.orientation.x = quat_x;
        odom_msg.pose.pose.orientation.y = quat_y;
        odom_msg.pose.pose.orientation.z = quat_z;
        odom_msg.pose.pose.orientation.w = quat_w;

        odom_pub_->publish(odom_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gps_to_odom>());
    rclcpp::shutdown();
    return 0;
}
