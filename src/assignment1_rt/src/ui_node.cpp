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
        publisher1_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        publisher2_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

    }
    void run(){
        while(rclcpp::ok())
        {
            getting_input();
            sendVelocity(std::stof(turtle));
        }
    }

    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher2_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string turtle;
    std::string linear_vel;
    std::string angular_vel;

    void sendVelocity(int turtle_choice){
        geometry_msgs::msg::Twist twist;
        twist.linear.x = std::stof(linear_vel);
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = std::stof(angular_vel);

        auto publisher = (turtle_choice == 1) ? publisher1_ : publisher2_;
    
        auto start_time = this->now();
        rclcpp::Rate rate(std::chrono::milliseconds(10));
        
        while ((this->now() - start_time).seconds() < 1.0) {
            publisher->publish(twist);
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        publisher->publish(twist);
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
    std::shared_ptr<UINode> node = std::make_shared<UINode>();
    node ->run();
    rclcpp::shutdown();
    return 0;


}