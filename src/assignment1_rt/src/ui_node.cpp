#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>


class UINode : public rclcpp::Node {

    public:
    UINode() : Node("ui_node"){
        RCLCPP_INFO(this->get_logger(), "Velocity Input Node has been started.");
        // getting_input();
        // std::string topic;
        // if(turtle.compare("1")==0) topic = "/turtle1";
        // if(turtle.compare("2")==0) turtle = "/turtle2";
        // topic = topic+ "/cmd_vel";
        // std::cout << topic << std::endl;
        publisher1_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        publisher2_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1),std::bind(&UINode::call_back, this) );
    }
    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher2_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string turtle;
    std::string linear_vel;
    std::string angular_vel;

    void call_back(){
        getting_input();
        if(turtle.compare("2")){
            geometry_msgs::msg::Twist msg;
            msg.linear.x = std::stof(linear_vel);
            msg.angular.x = std::stof(angular_vel);
            publisher2_->publish(msg);
            usleep(100000);
            geometry_msgs::msg::Twist msg_zero;
            publisher2_->publish(msg_zero);
        }
        if(turtle.compare("1")){
            geometry_msgs::msg::Twist msg;
            msg.linear.x = std::stof(linear_vel);
            msg.angular.x = std::stof(angular_vel);
            publisher1_->publish(msg);
            usleep(100000);
            geometry_msgs::msg::Twist msg_zero;
            publisher1_->publish(msg_zero);
        }

    }
    
    void getting_input(){
        std::cout << "Choose turtle: " << std::endl;
        std::cin >> turtle;
        std::cout << "Choose linear vel: " << std::endl; 
        std::cin >> linear_vel;
        std::cout << "Choose angular vel: " << std::endl; 
        std::cin >> angular_vel;
    };

};

int main(int argc, char * argv[]){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UINode>());
    rclcpp::shutdown();
    return 0;


}