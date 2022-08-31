#include <gtest/gtest.h>

#include "modulo_components/LifecycleComponent.hpp"
#include "test_modulo_components/communication_components.hpp"

using namespace modulo_components;

class LifecycleComponentCommunicationTest : public ::testing::Test {
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

TEST_F(LifecycleComponentCommunicationTest, InputOutput) {
  auto cartesian_state = state_representation::CartesianState::Random("test");
  auto input_node = std::make_shared<MinimalCartesianInput<LifecycleComponent>>(
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("period", 0.1)}), "/topic"
  );
  auto output_node = std::make_shared<MinimalCartesianOutput<LifecycleComponent>>(
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("period", 0.1)}), "/topic", cartesian_state
  );
  add_configure_activate(this->exec_, input_node);
  add_configure_activate(this->exec_, output_node);
  this->exec_->template spin_until_future_complete(input_node->received_future, 500ms);
  EXPECT_EQ(cartesian_state.get_name(), input_node->input->get_name());
  EXPECT_TRUE(cartesian_state.data().isApprox(input_node->input->data()));
  this->exec_.reset();
}
