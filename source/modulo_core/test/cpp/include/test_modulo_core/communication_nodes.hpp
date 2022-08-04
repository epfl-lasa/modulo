#pragma once

#include <rclcpp/rclcpp.hpp>

#include "modulo_core/communication/MessagePair.hpp"
#include "modulo_core/communication/PublisherHandler.hpp"
#include "modulo_core/communication/SubscriptionHandler.hpp"

using namespace std::chrono_literals;

namespace modulo_core::communication {

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
    this->received_future = this->received_.get_future();
    this->subscription_interface_ = std::make_shared<SubscriptionHandler<MsgT>>(message_pair);
    auto subscription = this->create_subscription<MsgT>(
        topic_name, 10, [this](const std::shared_ptr<MsgT> message) {
          this->subscription_interface_->template get_handler<MsgT>()->get_callback()(message);
          this->received_.set_value();
        }
    );
    this->subscription_interface_ =
        this->subscription_interface_->template get_handler<MsgT>()->create_subscription_interface(subscription);
  }

  std::shared_future<void> received_future;

private:
  std::promise<void> received_;
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

  void clear_nodes() {
    pub_node_.reset();
    sub_node_.reset();
  }

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::shared_ptr<rclcpp::Node> pub_node_;
  std::shared_ptr<rclcpp::Node> sub_node_;
  std::shared_ptr<rclcpp::Clock> clock_;
};

}// namespace modulo_core::communication
