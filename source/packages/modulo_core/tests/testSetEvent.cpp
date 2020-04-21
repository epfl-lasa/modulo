#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

using namespace std::chrono_literals;

class EventPublisher : public rclcpp::Node
{
  public:
    EventPublisher()
    : Node("event_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::Int64>("/event_id", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&EventPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      int event;
      auto message = std_msgs::msg::Int64();
      //std::cout << "Trigger event (3: start, 0: home, 1: action1, 2: action2): "; std::cin >> event;
      std::cout << "Trigger event (0: start, 1: launch, 1: action1, 2: action2): "; std::cin >> event;

      message.data = event;
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
    size_t count_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EventPublisher>());
    rclcpp::shutdown();
    return 0;
  }



