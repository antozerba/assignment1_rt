#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"


class UINode : public rclcpp::Node {

    public:
    UINode() : Node("ui_node"){
        RCLCPP_INFO(this->get_logger(), "Velocity Input Node has been started.");
        getting_input();
        std::string topic;
        if(turtle.compare("1")==0) topic = "/turtle1";
        if(turtle.compare("2")==0) turtle = "/turtle2";
        topic = topic+ "/cmd_vel";
        std::cout << topic << std::endl;
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic, 10);
    }
    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    std::string turtle;
    std::string vel;
    void getting_input(){
        std::cout << "Choose turtle: " << std::endl;
        std::cin >> turtle;
        std::cout << "Choose vel: " << std::endl; 
        std::cin >> vel;
    };
};

int main(int argc, char * argv[]){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UINode>());
    rclcpp::shutdown();
    return 0;


}