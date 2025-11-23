#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include <memory>
#include "std_msgs/msg/float32.hpp"
#include <chrono>
#include "geometry_msgs/msg/twist.hpp"

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
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&DistanceNode::distance_callback,this));

    }
    private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber1_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber2_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle1_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle2_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    turtlesim::msg::Pose pose1;
    turtlesim::msg::Pose pose2;
    double  distance;
    double tresh;

    void gettin_pose1(const turtlesim::msg::Pose::SharedPtr pose){
        pose1.x = pose -> x;
        pose1.y = pose -> y;
    }
    void gettin_pose2(const turtlesim::msg::Pose::SharedPtr pose){
        pose2.x = pose -> x;
        pose2.y = pose -> y;
    }
    void compute_distance(){
        distance = std::sqrt(std::pow(pose1.x - pose2.x, 2) +
                     std::pow(pose1.y - pose2.y, 2));
        RCLCPP_INFO(this->get_logger(), "DISTANCE COMPUTED");
    }
    void stopTurtle(int turtle_num) {
        auto stop_msg = geometry_msgs::msg::Twist();
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;

        if (turtle_num == 1) {
            turtle1_vel_pub_->publish(stop_msg);
        } else {
            turtle2_vel_pub_->publish(stop_msg);
        }

    }

    void distance_callback(){
        compute_distance();
        if(distance >0)
        {
            std_msgs::msg::Float32 msg;
            msg.data = distance;
            pub->publish(msg);
        }
        if (distance < tresh) {
                RCLCPP_INFO(this->get_logger(), "Turtles too close! Distance: %.2f", distance);
                stopTurtle(1);
                stopTurtle(2);
            }
    
    }


};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceNode>());
    rclcpp::shutdown();
    return 0;

}