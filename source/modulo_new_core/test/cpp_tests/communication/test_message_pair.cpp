#include <gtest/gtest.h>

#include "modulo_new_core/communication/MessagePair.hpp"

using namespace modulo_new_core::communication;

template<typename MsgT, typename DataT>
static void test_message_interface(
    MessageType type, const DataT& initial_value, const DataT& new_value, const std::shared_ptr<rclcpp::Clock> clock
) {
  auto data = std::make_shared<DataT>(initial_value);
  auto msg_pair = std::make_shared<MessagePair<MsgT, DataT>>(type, data, clock);
  EXPECT_EQ(initial_value, *msg_pair->get_data());
  EXPECT_EQ(initial_value, msg_pair->write_message().data);

  std::shared_ptr<MessagePairInterface> msg_pair_interface(msg_pair);
  auto msg = msg_pair_interface->template write<MsgT, DataT>();
  EXPECT_EQ(initial_value, msg.data);

  *data = new_value;
  EXPECT_EQ(new_value, *msg_pair->get_data());
  EXPECT_EQ(new_value, msg_pair->write_message().data);
  msg = msg_pair_interface->template write<MsgT, DataT>();
  EXPECT_EQ(new_value, msg.data);
}

class MessagePairTest : public ::testing::Test {
protected:
  void SetUp() override {
    clock_ = std::make_shared<rclcpp::Clock>();
  }

  std::shared_ptr<rclcpp::Clock> clock_;
};

TEST_F(MessagePairTest, TestBasicTypes) {
  test_message_interface<std_msgs::msg::Bool, bool>(MessageType::BOOL, false, true, clock_);
  test_message_interface<std_msgs::msg::Float64, double>(MessageType::FLOAT64, 0.1, 0.2, clock_);
  test_message_interface<std_msgs::msg::Float64MultiArray, std::vector<double>>(
      MessageType::FLOAT64_MULTI_ARRAY, {0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, clock_
  );
  test_message_interface<std_msgs::msg::Int32, int>(MessageType::INT32, 1, 2, clock_);
  test_message_interface<std_msgs::msg::String, std::string>(MessageType::STRING, "this", "that", clock_);
}

TEST_F(MessagePairTest, TestCartesianState) {
  auto initial_value = state_representation::CartesianState::Random("test");
  auto data = std::make_shared<state_representation::CartesianState>(initial_value);
  auto msg_pair = std::make_shared<MessagePair<modulo_new_core::EncodedState, state_representation::CartesianState>>(
      MessageType::ENCODED_STATE, data, clock_
  );
  EXPECT_TRUE(initial_value.data().isApprox(msg_pair->get_data()->data()));

  std::shared_ptr<MessagePairInterface> msg_pair_interface(msg_pair);
  auto msg = msg_pair_interface->write<modulo_new_core::EncodedState, state_representation::CartesianState>();
  std::string tmp(msg.data.begin(), msg.data.end());
  auto decoded = clproto::decode<state_representation::CartesianState>(tmp);
  EXPECT_TRUE(initial_value.data().isApprox(decoded.data()));

  auto new_value = state_representation::CartesianState::Identity("world");
  *data = new_value;
  msg = msg_pair_interface->write<modulo_new_core::EncodedState , state_representation::CartesianState>();
  tmp = std::string(msg.data.begin(), msg.data.end());
  decoded = clproto::decode<state_representation::CartesianState>(tmp);
  EXPECT_TRUE(new_value.data().isApprox(decoded.data()));
}
