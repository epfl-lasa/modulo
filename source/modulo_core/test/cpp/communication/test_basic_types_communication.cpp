#include <gtest/gtest.h>

#include "test_modulo_core/communication_nodes.hpp"

using namespace modulo_core::communication;

class BasicTypesCommunicationTest : public CommunicationTest {
protected:
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
};

TEST_F(BasicTypesCommunicationTest, BasicTypes) {
  this->communicate<std_msgs::msg::Bool, bool>(false, true);
  this->communicate<std_msgs::msg::Float64, double>(1.0, 2.0);
  this->communicate<std_msgs::msg::Float64MultiArray, std::vector<double>>({1.0, 2.0}, {3.0, 4.0});
  this->communicate<std_msgs::msg::Int32, int>(1, 2);
  this->communicate<std_msgs::msg::String, std::string>("this", "that");
}
