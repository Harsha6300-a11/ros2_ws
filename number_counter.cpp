#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class NumberCounter : public rclcpp::Node
{
public:
  NumberCounter()
  : Node("number_counter_cpp"), counter_(0)
  {
    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "number",
      10,
      std::bind(&NumberCounter::topic_callback, this, _1));
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("number_sum", 10);
    RCLCPP_INFO(this->get_logger(), "C++ Number Counter started");
  }

private:
  void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    counter_ += msg->data;
    auto out = std_msgs::msg::Int32();
    out.data = counter_;
    publisher_->publish(out);
    RCLCPP_INFO(this->get_logger(), "Current Sum: %d", out.data);
  }

  int32_t counter_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NumberCounter>());
  rclcpp::shutdown();
  return 0;
}
