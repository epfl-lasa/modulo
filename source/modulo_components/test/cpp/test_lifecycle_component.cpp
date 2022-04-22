#include <gtest/gtest.h>
#include <rclcpp/publisher.hpp>
#include <rclcpp/node_options.hpp>

#include "modulo_components/LifecycleComponent.hpp"
#include "modulo_new_core/EncodedState.hpp"

using namespace state_representation;

namespace modulo_components {

class LifecycleComponentPublicInterface : public LifecycleComponent {
public:
  explicit LifecycleComponentPublicInterface(const rclcpp::NodeOptions& node_options) : LifecycleComponent(node_options) {}
  using LifecycleComponent::add_output;
  using LifecycleComponent::configure_outputs;
  using LifecycleComponent::activate_outputs;
  using LifecycleComponent::outputs_;
  using LifecycleComponent::get_clock;
};

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
}

} // namespace modulo_components