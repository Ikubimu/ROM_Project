#include <chrono>   //milliseconds
#include <functional> // std::bind
#include <memory>
#include <string>
#include <math.h>
#include <vector>
#include <algorithm>

#include <csignal> //interruptions

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/twist.hpp"

#include "intelligent_mode/intelligent_mode.hpp"

using std::placeholders::_1;


enum States_robot
{
    MANUAL_MODE=0,
    RANDOM_MODE,
    INTELLIGENT_MODE
};

enum Movements_mode
{
    STOP = 0,
    GO_AHEAD,
    TURNING
};

class turtlebot3_main : public rclcpp::Node	
{

public: turtlebot3_main() : Node("turtlebot3_main")
{
    mode_subscription = this->create_subscription<std_msgs::msg::Int32>(
            "mode_listener", 
            10, 
            std::bind(&turtlebot3_main::state_callback, this, _1)
    );
    vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    init_MANUAL_MODE();
    //init_INTELLIGENT_MODE();
}

private:

void state_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    int a = msg->data;
    States_robot new_state = static_cast<States_robot>(a);

    if(new_state == state) return;

    using FuncPtr = void (turtlebot3_main::*)();
    FuncPtr func_init = nullptr;
    FuncPtr func_deinit = nullptr;


    switch(new_state){
        case MANUAL_MODE:
            func_init = &turtlebot3_main::init_MANUAL_MODE;
            break;
        case RANDOM_MODE:
            func_init = &turtlebot3_main::init_RANDOM_MODE;
            break;
        case INTELLIGENT_MODE:
            func_init = &turtlebot3_main::init_INTELLIGENT_MODE;
            break;
    }

    if(func_init == nullptr) return;

    switch(state){
        case MANUAL_MODE:
            func_deinit = &turtlebot3_main::deinit_MANUAL_MODE;
            break;
        case RANDOM_MODE:
            func_deinit = &turtlebot3_main::deinit_RANDOM_MODE;
            break;
        case INTELLIGENT_MODE:
            func_deinit = &turtlebot3_main::deinit_INTELLIGENT_MODE;
            break;
    }

    state = new_state;

    (this->*func_deinit)();
    (this->*func_init)();
    


}

void init_MANUAL_MODE()
{
    movements_subscription = this->create_subscription<std_msgs::msg::Int32>(
            "movements_listener", 
            10, 
            std::bind(&turtlebot3_main::update_movements, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "Current STATE is MANUAL_MODE");
}
void deinit_MANUAL_MODE()
{
    movements_subscription.reset();
}

void publish_cmd_vel(float lienar_vel, float ang_vel)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = lienar_vel;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = ang_vel;
    vel_publisher->publish(msg);
}

void update_movements(const std_msgs::msg::Int32::SharedPtr msg)
{
    int a = msg->data;
    Movements_mode mode = static_cast<Movements_mode>(a);

    switch(mode)
    {
        case STOP:
            publish_cmd_vel(0.0, 0.0);
            break;
        case GO_AHEAD:
            publish_cmd_vel(10.0, 0.0);
            break;
        case TURNING:
            publish_cmd_vel(0.0, 10.0);
            break;
    }
}

void amcl_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    x_rob = msg->pose.pose.position.x;
    y_rob = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    double roll, pitch, theta;
    tf2::Matrix3x3(q).getRPY(roll, pitch, theta);
    theta_rob = theta;
}

void get_next_point(geometry_msgs::msg::Point &goalPoint, 
                       std::vector<std::pair<double, double>> &vector_pos,
                       int& pos_index,
                       double& distance)
{
    distance = std::sqrt(
        std::pow(vector_pos[pos_index].first - x_rob, 2) +
        std::pow(vector_pos[pos_index].second - y_rob, 2)
    );
    double threshold = 0.1;
    if (distance < threshold) {
        pos_index = (pos_index + 1)% vector_pos.size();
        RCLCPP_INFO(this->get_logger(), "Node numer: %d, xrob: %f, yrob: %f", pos_index, x_rob, y_rob);
    }


    goalPoint.x = vector_pos[pos_index].first;
    goalPoint.y = vector_pos[pos_index].second;
}


void controller(){

    geometry_msgs::msg::Point goalPoint;
    static int pos_index = 1;
    double dClosest = 0.0;

    get_next_point(goalPoint, prm_path, pos_index, dClosest);
    double yl = -sin(theta_rob)*(goalPoint.x - x_rob) + cos(theta_rob)*(goalPoint.y - y_rob);
    double k = (2*yl)/(L*L);
    double v = (v_ref*dClosest)/L;;
    double w = v*k;
    if(pos_index == 0)
    {
        timer_controller.reset();
        RCLCPP_INFO(this->get_logger(), "TARGET REACHED OMG");
        v = 0;
        w = 0;
    }
    publish_cmd_vel(v, w);

}

void init_RANDOM_MODE()
{

}
void deinit_RANDOM_MODE()
{

}


void init_INTELLIGENT_MODE()
{
    RCLCPP_INFO(this->get_logger(), "Current STATE is INTELLIGENT_MODE");

    vector_pos = {
        {0, 0}, {0.25, 0}, {0.12, 0.12}
    };

    get_prm_path(prm_path, 100, 5, {x_rob, y_rob}, {2, 2.5}); //example values

    //prm_path = vector_pos;

    acml_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "/amcl_odom",10,std::bind(&turtlebot3_main::amcl_callback,this,std::placeholders::_1));
    
    std::chrono::milliseconds period_control = std::chrono::milliseconds(50);
    timer_controller = this->create_wall_timer(period_control,std::bind(&turtlebot3_main::controller,this));

}


void deinit_INTELLIGENT_MODE()
{
    acml_subscriber.reset();
    timer_controller.reset();
}

rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_subscription;
rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr movements_subscription;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr acml_subscriber;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;


States_robot state = MANUAL_MODE;
std::vector<std::pair<double, double>> prm_path;

rclcpp::TimerBase::SharedPtr timer_controller;
double x_rob{0.0},y_rob{0.0},theta_rob{0.0};
double b = (0.16/2);
double L=0.25;
double v_ref = 0.1;
std::vector<std::pair<double, double>> vector_pos; //example positions for the robot
};

std::shared_ptr<turtlebot3_main> node = nullptr;


int main(int argc, char *argv[]){

    rclcpp::init(argc,argv);

    node = std::make_shared<turtlebot3_main>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
