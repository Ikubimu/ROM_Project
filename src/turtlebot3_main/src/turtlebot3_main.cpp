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

    //init_MANUAL_MODE();
    init_INTELLIGENT_MODE();
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

void init_RANDOM_MODE()
{

}
void deinit_RANDOM_MODE()
{

}

void init_INTELLIGENT_MODE()
{
    get_prm_path(prm_path, 100, 5, {0,0}, {3, 2}); //example values
}
void deinit_INTELLIGENT_MODE()
{

}

rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_subscription;
rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr movements_subscription;

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;


States_robot state = MANUAL_MODE;
std::vector<std::pair<double, double>> prm_path;
};

std::shared_ptr<turtlebot3_main> node = nullptr;


int main(int argc, char *argv[]){

    rclcpp::init(argc,argv);

    node = std::make_shared<turtlebot3_main>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
