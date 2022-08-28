#include <gtest/gtest.h>

#include "modulo_components/Component.hpp"
#include "test_modulo_components/communication_components.hpp"

using namespace modulo_components;

class ComponentCommunicationTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
};

TEST_F(ComponentCommunicationTest, InputOutput) {
  auto cartesian_state = state_representation::CartesianState::Random("test");
  auto input_node = std::make_shared<MinimalInput<Component>>(
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("period", 0.1)}), "/topic"
  );
  auto output_node = std::make_shared<MinimalOutput<Component>>(
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("period", 0.1)}), "/topic", cartesian_state
  );
  this->exec_->add_node(input_node);
  this->exec_->add_node(output_node);
  this->exec_->template spin_until_future_complete(input_node->received_future, 500ms);
  EXPECT_EQ(cartesian_state.get_name(), input_node->input->get_name());
  EXPECT_TRUE(cartesian_state.data().isApprox(input_node->input->data()));
  this->exec_.reset();
}
