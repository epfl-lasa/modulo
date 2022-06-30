#include <gtest/gtest.h>

#include <rclcpp/node_options.hpp>

#include "modulo_core/EncodedState.hpp"
#include "test_modulo_components/component_public_interfaces.hpp"

using namespace state_representation;

namespace modulo_components {

class LifecycleComponentTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    component_ = std::make_shared<LifecycleComponentPublicInterface>(rclcpp::NodeOptions());
  }

  std::shared_ptr<LifecycleComponentPublicInterface> component_;
};

TEST_F(LifecycleComponentTest, AddOutput) {
  std::shared_ptr<State> data = make_shared_state(CartesianState::Random("test"));
  component_->add_output("test", data, true);
  auto outputs_iterator = component_->outputs_.find("test");
  EXPECT_TRUE(outputs_iterator != component_->outputs_.end());
  EXPECT_NO_THROW(component_->configure_outputs());
  EXPECT_NO_THROW(component_->activate_outputs());
  EXPECT_NO_THROW(component_->outputs_.at("test")->publish());

  auto new_data = std::make_shared<bool>(false);
  component_->add_output("test", new_data, true);
  EXPECT_EQ(component_->outputs_.at("test")->get_message_pair()->get_type(),
            modulo_core::communication::MessageType::ENCODED_STATE);
}
} // namespace modulo_components
