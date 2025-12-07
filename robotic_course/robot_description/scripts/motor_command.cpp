#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class MotorCommandNode : public rclcpp::Node
{
public:
    MotorCommandNode() : Node("cmd_vel_to_motor")
    {
        wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.1);
        wheel_separation_ = this->declare_parameter<double>("wheel_separation", 0.45);
        std::string cmd_vel_topic = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

        left_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("/motor_left_rpm", 10);
        right_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("/motor_right_rpm", 10);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic,
            rclcpp::QoS(10),
            std::bind(&MotorCommandNode::cmdVelCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "cmd_vel_to_motor started (r=%.3f, w=%.3f)", wheel_radius_, wheel_separation_);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double v = msg->linear.x;
        double w = msg->angular.z;

        std_msgs::msg::Float64 left_msg;
        std_msgs::msg::Float64 right_msg;

        // rad/s
        double omega_left = (2.0 * v - w * wheel_separation_) / (2.0 * wheel_radius_);
        double omega_right = (2.0 * v + w * wheel_separation_) / (2.0 * wheel_radius_);

        // convert to RPM
        const double radps_to_rpm = 60.0 / (2.0 * M_PI);
        left_msg.data = omega_left * radps_to_rpm;
        right_msg.data = omega_right * radps_to_rpm;

        left_motor_pub_->publish(left_msg);
        right_motor_pub_->publish(right_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_motor_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    double wheel_radius_;
    double wheel_separation_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorCommandNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
