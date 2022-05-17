#include <gtest/gtest.h>

#include "modulo_new_core/communication/MessagePair.hpp"

using namespace modulo_new_core::communication;

template<typename MsgT, typename DataT>
static void test_message_interface(
    const DataT& initial_value, const DataT& new_value, const std::shared_ptr<rclcpp::Clock> clock
) {
  auto data = std::make_shared<DataT>(initial_value);
  auto msg_pair = std::make_shared<MessagePair<MsgT, DataT>>(data, clock);
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

  msg = MsgT();
  msg.data = initial_value;
  msg_pair_interface->template read<MsgT, DataT>(msg);
  EXPECT_EQ(initial_value, *msg_pair->get_data());
}

class MessagePairTest : public ::testing::Test {
protected:
  void SetUp() override {
    clock_ = std::make_shared<rclcpp::Clock>();
  }

  std::shared_ptr<rclcpp::Clock> clock_;
};

TEST_F(MessagePairTest, BasicTypes) {
  test_message_interface<std_msgs::msg::Bool, bool>(false, true, clock_);
  test_message_interface<std_msgs::msg::Float64, double>(0.1, 0.2, clock_);
  test_message_interface<std_msgs::msg::Float64MultiArray, std::vector<double>>(
      {0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, clock_
  );
  test_message_interface<std_msgs::msg::Int32, int>(1, 2, clock_);
  test_message_interface<std_msgs::msg::String, std::string>("this", "that", clock_);
}

TEST_F(MessagePairTest, EncodedState) {
  auto initial_value = state_representation::CartesianState::Random("test");
  auto data = state_representation::make_shared_state(initial_value);
  auto msg_pair =
      std::make_shared<MessagePair<modulo_new_core::EncodedState, state_representation::State>>(data, clock_);
  EXPECT_TRUE(initial_value.data().isApprox(
      std::dynamic_pointer_cast<state_representation::CartesianState>(msg_pair->get_data())->data()));

  std::shared_ptr<MessagePairInterface> msg_pair_interface(msg_pair);
  auto msg = msg_pair_interface->write<modulo_new_core::EncodedState, state_representation::State>();
  std::string tmp(msg.data.begin(), msg.data.end());
  auto decoded = clproto::decode<state_representation::CartesianState>(tmp);
  EXPECT_TRUE(initial_value.data().isApprox(decoded.data()));

  auto new_value = state_representation::CartesianState::Identity("world");
  msg_pair->set_data(state_representation::make_shared_state(new_value));
  msg = msg_pair_interface->write<modulo_new_core::EncodedState, state_representation::State>();
  tmp = std::string(msg.data.begin(), msg.data.end());
  decoded = clproto::decode<state_representation::CartesianState>(tmp);
  EXPECT_TRUE(new_value.data().isApprox(decoded.data()));

  data = state_representation::make_shared_state(initial_value);
  msg_pair->set_data(data);
  msg = modulo_new_core::EncodedState();
  modulo_new_core::translators::write_msg(msg, data, clock_->now());
  msg_pair_interface->read<modulo_new_core::EncodedState, state_representation::State>(msg);
  EXPECT_TRUE(initial_value.data().isApprox(
      std::dynamic_pointer_cast<state_representation::CartesianState>(msg_pair->get_data())->data()));
}
