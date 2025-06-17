#ifndef RANDOM_MODE_HPP_
#define RANDOM_MODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class RandomMode
{
public:
    RandomMode(rclcpp::Node * node, std::function<void(double, double)> publish_cmd_vel);

    void start();
    void stop();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void loop();

    rclcpp::Node * node_;
    std::function<void(double, double)> publish_cmd_vel_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry current_odom_;
    geometry_msgs::msg::PoseWithCovarianceStamped current_amcl_;

    bool is_turning_;
    rclcpp::Time last_turn_time_;
    rclcpp::Duration turning_duration_;
    double collision_threshold_;
};

#endif // RANDOM_MODE_HPP_