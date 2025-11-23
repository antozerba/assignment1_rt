#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include <memory>
#include "std_msgs/msg/float32.hpp"
#include <chrono>

class DistanceNode : public rclcpp::Node {
    public:
    DistanceNode() : Node("distance_node")
    {
        RCLCPP_INFO(this-> get_logger(),"NODE CREATED");
        subscriber1_ = this->create_subscription<turtlesim::msg::Pose>
        ("/turtle1/pose", 10, std::bind(&DistanceNode::gettin_pose1, this, std::placeholders::_1));
        subscriber2_ = this->create_subscription<turtlesim::msg::Pose>
        ("/turtle2/pose", 10, std::bind(&DistanceNode::gettin_pose2, this, std::placeholders::_1));
        pub = this->create_publisher<std_msgs::msg::Float32>("/distance", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&DistanceNode::compute_distance,this));

    }
    private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber1_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber2_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer_;
    turtlesim::msg::Pose pose1;
    turtlesim::msg::Pose pose2;
    std_msgs::msg::Float32 distance;

    void gettin_pose1(const turtlesim::msg::Pose::SharedPtr pose){
        pose1.x = pose -> x;
        pose1.y = pose -> y;
    }
    void gettin_pose2(const turtlesim::msg::Pose::SharedPtr pose){
        pose2.x = pose -> x;
        pose2.y = pose -> y;
    }
    void compute_distance(){
        std_msgs::msg::Float32 msg;
        msg.data = std::sqrt(std::pow(pose1.x - pose2.x, 2) +
                     std::pow(pose1.y - pose2.y, 2));
        RCLCPP_INFO(this->get_logger(), "DISTANCE COMPUTED");
        pub ->publish(msg);
    }


};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceNode>());
    rclcpp::shutdown();
    return 0;

}