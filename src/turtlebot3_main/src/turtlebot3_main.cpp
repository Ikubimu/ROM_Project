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
#include <sensor_msgs/msg/laser_scan.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

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
    RIGHT,
    LEFT
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
    subscription_map = this->create_subscription<std_msgs::msg::String>(
        "/map_path",
        10,
        std::bind(&turtlebot3_main::map_callback, this, _1)
    );
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
//function to publish cmd_vel, common in all states
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


/*********************************************
************FUNCTION MANUAL MODE**************
**********************************************/

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
        case RIGHT:
            publish_cmd_vel(0.0, -10.0);
            break;
        case LEFT:
            publish_cmd_vel(0.0, 10.0);
            break;
    }
}

/*********************************************
************FUNCTION INTELLIGENT MODE*********
**********************************************/

//callback to assign map.yalm path from acml node
void map_callback(std_msgs::msg::String::SharedPtr msg)
{
    map_path = msg->data.c_str();
}

//get pos and theta alredy calculated from acml node
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
    RCLCPP_INFO(this->get_logger(), "xrob: %f, yrob: %f", x_rob, y_rob);
    theta_rob = theta;
}

//get next point to follow
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

//function is periodically called up for motion control
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



void init_INTELLIGENT_MODE()
{
    RCLCPP_INFO(this->get_logger(), "Current STATE is INTELLIGENT_MODE");

    if(map_path == "")
    {
        state = MANUAL_MODE;
        RCLCPP_INFO(this->get_logger(), "There is not map to travel");
        init_MANUAL_MODE();
    }

    std::vector<std::pair<double, double>> vector_pos; //example positions for the robot for testing
    vector_pos = {
        {0, 0}, {0.75, 0}, {0.75, 1}
    };

    get_prm_path(map_path, prm_path, 200, 5, {x_rob, y_rob}, {0.7, 0.7}); //example values

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


/*********************************************
************FUNCTION RANDOM MODE**************
**********************************************/

void change_position()
{
    publish_cmd_vel(((float)(rand()%100) * 0.1 -5.0), ((float)(rand()%100) * 0.1 -5.0));
    RCLCPP_INFO(this->get_logger(), "Its random day");
}

void turn_random()
{

    timer_random.reset();
    publish_cmd_vel(1.0, 0.0);
    sub_lidar = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&turtlebot3_main::laser_callback, this, _1)
    );

}

void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    for(int i=0; i<10; i++)
    {

        float dist = msg->ranges[0];
        if(dist <= 0.2)
        {
            publish_cmd_vel(0.0, 5.0);
            sub_lidar.reset();
            timer_random = this->create_wall_timer(std::chrono::seconds(rand()%5 +2), std::bind(&turtlebot3_main::turn_random,this));
            return;
        }
    }
}

void init_RANDOM_MODE()
{
    RCLCPP_INFO(this->get_logger(), "Current STATE is RANDOM_MODE");
    sub_lidar = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&turtlebot3_main::laser_callback, this, _1)
    );
    publish_cmd_vel(1.0, 0.0);
}
void deinit_RANDOM_MODE()
{
    timer_random.reset();
    sub_lidar.reset();
}



States_robot state = MANUAL_MODE;

rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_subscription;
rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr movements_subscription;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr acml_subscriber;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_map;

std::string map_path = "";
std::vector<std::pair<double, double>> prm_path;
rclcpp::TimerBase::SharedPtr timer_controller;
double x_rob{0.0},y_rob{0.0},theta_rob{0.0};
double b = (0.16/2);
double L=0.25;
double v_ref = 0.05;


rclcpp::TimerBase::SharedPtr timer_random;
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar;


};

std::shared_ptr<turtlebot3_main> node = nullptr;


int main(int argc, char *argv[]){

    rclcpp::init(argc,argv);

    node = std::make_shared<turtlebot3_main>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
