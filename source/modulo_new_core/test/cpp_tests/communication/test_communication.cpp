#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "modulo_new_core/communication/MessagePair.hpp"
#include "modulo_new_core/communication/PublisherHandler.hpp"
#include "modulo_new_core/communication/SubscriptionHandler.hpp"

using namespace std::chrono_literals;
using namespace modulo_new_core::communication;

template<typename MsgT>
class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher(const std::string& topic_name, std::shared_ptr<MessagePairInterface> message_pair) :
      Node("minimal_publisher") {
    auto publisher = this->create_publisher<MsgT>(topic_name, 10);
    this->publisher_interface_ = std::make_shared<PublisherHandler<rclcpp::Publisher<MsgT>, MsgT>>(
        PublisherType::PUBLISHER, publisher
    )->create_publisher_interface(message_pair);
    timer_ = this->create_wall_timer(10ms, [this]() { this->publisher_interface_->publish(); });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<PublisherInterface> publisher_interface_;
};

template<typename MsgT>
class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber(const std::string& topic_name, std::shared_ptr<MessagePairInterface> message_pair) :
      Node("minimal_subscriber") {
    auto subscription_handler = std::make_shared<SubscriptionHandler<MsgT>>(message_pair);
    auto subscription = this->create_subscription<MsgT>(topic_name, 10, subscription_handler->get_callback());
    this->subscription_interface_ = subscription_handler->create_subscription_interface(subscription);
  }

  MinimalSubscriber(const std::string& topic_name, std::function<void(std::shared_ptr<MsgT>)> callback) :
      Node("minimal_subscriber") {
    auto subscription = this->create_subscription<MsgT>(topic_name, 10, callback);
    this->subscription_interface_ =
        std::make_shared<SubscriptionHandler<MsgT>>()->create_subscription_interface(subscription);
  }

private:
  std::shared_ptr<SubscriptionInterface> subscription_interface_;
};

class CommunicationTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);

    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    clock_ = std::make_shared<rclcpp::Clock>();
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  template<typename MsgT>
  void add_nodes(
      const std::string& topic_name, const std::shared_ptr<MessagePairInterface>& pub_message,
      const std::shared_ptr<MessagePairInterface>& sub_message
  ) {
    pub_node_ = std::make_shared<MinimalPublisher<MsgT>>(topic_name, pub_message);
    sub_node_ = std::make_shared<MinimalSubscriber<MsgT>>(topic_name, sub_message);
    exec_->add_node(pub_node_);
    exec_->add_node(sub_node_);
  }

  template<typename MsgT>
  void add_nodes(
      const std::string& topic_name, const std::shared_ptr<MessagePairInterface>& pub_message,
      std::function<void(std::shared_ptr<MsgT>)> callback
  ) {
    pub_node_ = std::make_shared<MinimalPublisher<MsgT>>(topic_name, pub_message);
    sub_node_ = std::make_shared<MinimalSubscriber<MsgT>>(topic_name, callback);
    exec_->add_node(pub_node_);
    exec_->add_node(sub_node_);
  }

  void clear_nodes() {
    pub_node_.reset();
    sub_node_.reset();
  }

  template<typename MsgT, typename DataT>
  void communicate(const DataT& initial_value, const DataT& new_value) {
    auto pub_data = std::make_shared<DataT>(new_value);
    auto pub_message = make_shared_message_pair(pub_data, this->clock_);
    auto sub_data = std::make_shared<DataT>(initial_value);
    auto sub_message = make_shared_message_pair(sub_data, this->clock_);
    this->add_nodes<MsgT>("/test_topic", pub_message, sub_message);

    for (std::size_t i = 0; i < 20; ++i) {
      this->exec_->spin_once();
    }

    EXPECT_EQ(*pub_data, *sub_data);
    this->clear_nodes();
  }

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::shared_ptr<rclcpp::Node> pub_node_;
  std::shared_ptr<rclcpp::Node> sub_node_;
  std::shared_ptr<rclcpp::Clock> clock_;
};

TEST_F(CommunicationTest, BasicTypes) {
  this->communicate<std_msgs::msg::Bool, bool>(false, true);
  this->communicate<std_msgs::msg::Float64, double>(1.0, 2.0);
  this->communicate<std_msgs::msg::Float64MultiArray, std::vector<double>>({1.0, 2.0}, {3.0, 4.0});
  this->communicate<std_msgs::msg::Int32, int>(1, 2);
  this->communicate<std_msgs::msg::String, std::string>("this", "that");
}
