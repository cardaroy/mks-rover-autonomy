#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class SumSubscriber : public rclcpp::Node
{
public:
  SumSubscriber() : Node("sum_subscriber"), sum_(0)
  {
    sub_ = create_subscription<std_msgs::msg::Int32>(
      "numbers", 10,
      std::bind(&SumSubscriber::callback, this, std::placeholders::_1));
  }

private:
  void callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    sum_ += msg->data;
    RCLCPP_INFO(get_logger(), "Received: %d | Sum: %d", msg->data, sum_);
  }

  int sum_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SumSubscriber>());
  rclcpp::shutdown();
  return 0;
}
