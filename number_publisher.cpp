#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class NumberPublisher : public rclcpp::Node
{
public:
  NumberPublisher()
  : Node("number_publisher_cpp"), number_(3)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("number", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&NumberPublisher::publish_number, this));
    RCLCPP_INFO(this->get_logger(), "C++ Number Publisher started");
  }

private:
  void publish_number()
  {
    auto msg = std_msgs::msg::Int32();
    msg.data = number_;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing: %d", msg.data);
  }

  int32_t number_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NumberPublisher>());
  rclcpp::shutdown();
  return 0;
}
