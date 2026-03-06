#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class NumberPublisher : public rclcpp::Node
{
public:
  NumberPublisher() : Node("number_publisher"), count_(0)
  {
    pub_ = create_publisher<std_msgs::msg::Int32>("numbers", 10);
    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&NumberPublisher::publish_number, this));
  }

private:
  void publish_number()
  {
    std_msgs::msg::Int32 msg;
    msg.data = count_++;
    pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published: %d", msg.data);
  }

  int count_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NumberPublisher>());
  rclcpp::shutdown();
  return 0;
}
