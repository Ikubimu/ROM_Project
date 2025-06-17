#include "random_mode.hpp"

RandomMode::RandomMode(rclcpp::Node * node, std::function<void(double, double)> publish_cmd_vel)
: node_(node),
  publish_cmd_vel_(publish_cmd_vel),
  is_turning_(false),
  turning_duration_(rclcpp::Duration::from_seconds(2.0)),
  collision_threshold_(0.5)
{
}

void RandomMode::start()
{
    odom_subscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&RandomMode::odom_callback, this, std::placeholders::_1));

    amcl_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "amcl_pose", 10, std::bind(&RandomMode::amcl_callback, this, std::placeholders::_1));

    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RandomMode::loop, this)
    );

    is_turning_ = false;
    last_turn_time_ = node_->now();

    RCLCPP_INFO(node_->get_logger(), "RandomMode started");
}

void RandomMode::stop()
{
    odom_subscription_.reset();
    amcl_subscription_.reset();
    timer_.reset();
    RCLCPP_INFO(node_->get_logger(), "RandomMode stopped");
}

void RandomMode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_odom_ = *msg;
}

void RandomMode::amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    current_amcl_ = *msg;
}

void RandomMode::loop()
{
    double odom_x = current_odom_.pose.pose.position.x;
    double odom_y = current_odom_.pose.pose.position.y;

    double amcl_x = current_amcl_.pose.pose.position.x;
    double amcl_y = current_amcl_.pose.pose.position.y;

    double dx = odom_x - amcl_x;
    double dy = odom_y - amcl_y;
    double dist = std::sqrt(dx*dx + dy*dy);

    if (!is_turning_)
    {
        if (dist > collision_threshold_)
        {
            is_turning_ = true;
            last_turn_time_ = node_->now();
            publish_cmd_vel_(0.0, 0.5);
            RCLCPP_INFO(node_->get_logger(), "COLLISION detected! Turning...");
        }
        else
        {
            publish_cmd_vel_(0.1, 0.0);
        }
    }
    else
    {
        auto now = node_->now();
        if ((now - last_turn_time_) > turning_duration_)
        {
            is_turning_ = false;
            publish_cmd_vel_(0.1, 0.0);
            RCLCPP_INFO(node_->get_logger(), "Finished turning. Moving forward...");
        }
    }
}