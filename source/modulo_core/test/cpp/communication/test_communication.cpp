#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

#include "modulo_core/communication/MessagePair.hpp"
#include "modulo_core/communication/PublisherHandler.hpp"
#include "modulo_core/communication/SubscriptionHandler.hpp"

using namespace std::chrono_literals;
using namespace modulo_core::communication;

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

  template<typename MsgT, typename DataT>
  void communicate(const DataT& initial_value, const DataT& new_value) {
    auto pub_data = std::make_shared<DataT>(new_value);
    auto pub_message = make_shared_message_pair(pub_data, this->clock_);
    auto sub_data = std::make_shared<DataT>(initial_value);
    auto sub_message = make_shared_message_pair(sub_data, this->clock_);
    this->add_nodes<MsgT>("/test_topic", pub_message, sub_message);

    this->exec_->template spin_until_future_complete(
        std::dynamic_pointer_cast<MinimalSubscriber<MsgT>>(this->sub_node_)->received_future, 500ms
    );

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

TEST_F(CommunicationTest, EncodedStateSameType) {
  using namespace state_representation;
  auto pub_state = std::make_shared<CartesianState>(CartesianState::Random("this", "world"));
  auto pub_message = make_shared_message_pair(pub_state, this->clock_);
  auto sub_state = std::make_shared<CartesianState>(CartesianState::Identity("that", "base"));
  auto sub_message = make_shared_message_pair(sub_state, this->clock_);
  this->add_nodes<modulo_core::EncodedState>("/test_topic", pub_message, sub_message);
  this->exec_->template spin_until_future_complete(
      std::dynamic_pointer_cast<MinimalSubscriber<modulo_core::EncodedState>>(this->sub_node_)->received_future, 500ms
  );

  EXPECT_EQ(pub_state->get_type(), sub_state->get_type());
  EXPECT_EQ(pub_state->get_name(), sub_state->get_name());
  EXPECT_EQ(pub_state->get_reference_frame(), sub_state->get_reference_frame());
  EXPECT_TRUE(pub_state->data().isApprox(sub_state->data()));
}

TEST_F(CommunicationTest, EncodedStateIncompatibleType) {
  using namespace state_representation;
  auto pub_state = std::make_shared<CartesianPose>(CartesianPose::Random("this", "world"));
  auto pub_message = make_shared_message_pair(pub_state, this->clock_);
  auto sub_state = std::make_shared<JointState>(JointState::Zero("that", 3));
  auto sub_message = make_shared_message_pair(sub_state, this->clock_);
  this->add_nodes<modulo_core::EncodedState>("/test_topic", pub_message, sub_message);
  this->exec_->template spin_until_future_complete(
      std::dynamic_pointer_cast<MinimalSubscriber<modulo_core::EncodedState>>(this->sub_node_)->received_future, 500ms
  );

  // An exception is thrown and caught, so the sub state didn't change
  EXPECT_EQ(sub_state->get_type(), StateType::JOINT_STATE);
  EXPECT_TRUE(sub_state->data().isApprox(JointState::Zero("that", 3).data()));
}

TEST_F(CommunicationTest, EncodedStateCompatibleType) {
  using namespace state_representation;
  auto pub_state = std::make_shared<JointState>(JointState::Random("this", 3));
  auto pub_message = make_shared_message_pair(pub_state, this->clock_);
  auto sub_state = std::make_shared<JointPositions>(JointPositions::Zero("that", 3));
  auto sub_message = make_shared_message_pair(sub_state, this->clock_);
  this->add_nodes<modulo_core::EncodedState>("/test_topic", pub_message, sub_message);
  this->exec_->template spin_until_future_complete(
      std::dynamic_pointer_cast<MinimalSubscriber<modulo_core::EncodedState>>(this->sub_node_)->received_future, 500ms
  );

  EXPECT_EQ(sub_state->get_type(), StateType::JOINT_POSITIONS);
  EXPECT_EQ(sub_state->get_name(), pub_state->get_name());
  EXPECT_TRUE(sub_state->get_positions().isApprox(pub_state->get_positions()));
}

TEST_F(CommunicationTest, EncodedStateCompatibleType2) {
  using namespace state_representation;
  auto pub_state = std::make_shared<JointPositions>(JointPositions::Random("this", 3));
  auto pub_message = make_shared_message_pair(pub_state, this->clock_);
  auto sub_state = std::make_shared<JointState>(JointState::Zero("that", 3));
  auto sub_message = make_shared_message_pair(sub_state, this->clock_);
  this->add_nodes<modulo_core::EncodedState>("/test_topic", pub_message, sub_message);
  this->exec_->template spin_until_future_complete(
      std::dynamic_pointer_cast<MinimalSubscriber<modulo_core::EncodedState>>(this->sub_node_)->received_future, 500ms
  );

  EXPECT_EQ(sub_state->get_type(), StateType::JOINT_STATE);
  EXPECT_EQ(sub_state->get_name(), pub_state->get_name());
  EXPECT_TRUE(sub_state->get_positions().isApprox(pub_state->get_positions()));
}

TEST_F(CommunicationTest, EncodedStateCompatibleType3) {
  using namespace state_representation;
  auto pub_state = std::make_shared<CartesianState>(CartesianState::Random("this"));
  auto pub_message = make_shared_message_pair(pub_state, this->clock_);
  auto sub_state = std::make_shared<CartesianPose>(CartesianPose::Identity("that"));
  auto sub_message = make_shared_message_pair(sub_state, this->clock_);
  this->add_nodes<modulo_core::EncodedState>("/test_topic", pub_message, sub_message);
  this->exec_->template spin_until_future_complete(
      std::dynamic_pointer_cast<MinimalSubscriber<modulo_core::EncodedState>>(this->sub_node_)->received_future, 500ms
  );

  EXPECT_EQ(sub_state->get_type(), StateType::CARTESIAN_POSE);
  EXPECT_EQ(sub_state->get_name(), pub_state->get_name());
  EXPECT_TRUE(sub_state->get_pose().isApprox(pub_state->get_pose()));
}

TEST_F(CommunicationTest, EncodedStateCompatibleType4) {
  using namespace state_representation;
  auto pub_state = std::make_shared<CartesianPose>(CartesianPose::Random("this"));
  auto pub_message = make_shared_message_pair(pub_state, this->clock_);
  auto sub_state = std::make_shared<CartesianState>(CartesianState::Identity("that"));
  auto sub_message = make_shared_message_pair(sub_state, this->clock_);
  EXPECT_NE(sub_state->get_type(), pub_state->get_type());
  this->add_nodes<modulo_core::EncodedState>("/test_topic", pub_message, sub_message);
  this->exec_->template spin_until_future_complete(
      std::dynamic_pointer_cast<MinimalSubscriber<modulo_core::EncodedState>>(this->sub_node_)->received_future, 500ms
  );

  EXPECT_EQ(sub_state->get_type(), StateType::CARTESIAN_STATE);
  EXPECT_EQ(sub_state->get_name(), pub_state->get_name());
  EXPECT_TRUE(sub_state->get_pose().isApprox(pub_state->get_pose()));
}
