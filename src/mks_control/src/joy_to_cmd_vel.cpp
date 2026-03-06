#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class JoyToCmdVel : public rclcpp::Node
{
public:
    JoyToCmdVel() : Node("joy_to_cmd_vel")
    {
        this->declare_parameter<double>("max_linear_speed", 1.0);
        this->declare_parameter<double>("max_angular_speed", 1.0);

        max_lin_ = this->get_parameter("max_linear_speed").as_double();
        max_ang_ = this->get_parameter("max_angular_speed").as_double();

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&JoyToCmdVel::joyCallback, this, std::placeholders::_1)
        );

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10
        );
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Joy message missing axes");
            return;
        }

        geometry_msgs::msg::Twist cmd;

        // Same mapping you already had
        cmd.linear.x  = msg->axes[1] * max_lin_;
        cmd.angular.z = msg->axes[0] * max_ang_;

        cmd_vel_pub_->publish(cmd);
    }

    double max_lin_;
    double max_ang_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToCmdVel>());
    rclcpp::shutdown();
    return 0;
}