#include <gtest/gtest.h>

#include "modulo_new_core/communication/MessagePair.hpp"

using namespace modulo_new_core::communication;

template<typename MsgT, typename DataT>
static void test_message_interface(const DataT& initial_value, const DataT& new_value) {
  auto data = std::make_shared<DataT>(initial_value);
  auto msg_pair = std::make_shared<MessagePair<MsgT, DataT>>(data);
  EXPECT_EQ(initial_value, *msg_pair->get_data());
  EXPECT_EQ(initial_value, msg_pair->write_message().data);

  std::shared_ptr<MessagePairInterface> msg_pair_interface(msg_pair);
  auto msg = msg_pair_interface->template write_message<MsgT, DataT>();
  EXPECT_EQ(initial_value, msg.data);

  *data = new_value;
  EXPECT_EQ(new_value, *msg_pair->get_data());
  EXPECT_EQ(new_value, msg_pair->write_message().data);
  msg = msg_pair_interface->template write_message<MsgT, DataT>();
  EXPECT_EQ(new_value, msg.data);
}

TEST(MessagePairTest, TestBasicTypes) {
  test_message_interface<std_msgs::msg::Bool, bool>(false, true);
  test_message_interface<std_msgs::msg::Float64, double>(0.1, 0.2);
  test_message_interface<std_msgs::msg::Int64, int>(1, 2);
  test_message_interface<std_msgs::msg::String, std::string>("this", "that");
}

TEST(MessagePairTest, TestDoubleArray) {
  std::vector<double> initial_value = {0.1, 0.2, 0.3};
  auto data = std::make_shared<std::vector<double>>(initial_value);
  auto msg_pair = std::make_shared<MessagePair<std_msgs::msg::Float64MultiArray, std::vector<double>>>(data);
  for (std::size_t i = 0; i < initial_value.size(); ++i) {
    EXPECT_EQ(initial_value.at(i), msg_pair->get_data()->at(i));
    EXPECT_EQ(initial_value.at(i), msg_pair->write_message().data.at(i));
  }

  std::shared_ptr<MessagePairInterface> msg_pair_interface(msg_pair);
  auto msg = msg_pair_interface->write_message<std_msgs::msg::Float64MultiArray, std::vector<double>>();
  for (std::size_t i = 0; i < initial_value.size(); ++i) {
    EXPECT_EQ(initial_value.at(i),msg.data.at(i));
  }
  EXPECT_EQ(initial_value, msg.data);

  std::vector<double> new_value = {0.4, 0.5};
  *data = new_value;
  msg = msg_pair_interface->write_message<std_msgs::msg::Float64MultiArray, std::vector<double>>();
  for (std::size_t i = 0; i < new_value.size(); ++i) {
    EXPECT_EQ(new_value.at(i), msg_pair->get_data()->at(i));
    EXPECT_EQ(new_value.at(i), msg_pair->write_message().data.at(i));
    EXPECT_EQ(new_value.at(i),msg.data.at(i));
  }
}
