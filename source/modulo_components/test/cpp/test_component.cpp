#include <gtest/gtest.h>
#include <rclcpp/publisher.hpp>
#include <rclcpp/node_options.hpp>

#include "modulo_components/Component.hpp"
#include "modulo_new_core/EncodedState.hpp"

using namespace state_representation;

namespace modulo_components {

class ComponentPublicInterface : public Component {
public:
  explicit ComponentPublicInterface(const rclcpp::NodeOptions& node_options) : Component(node_options) {}
  using Component::add_output;
  using Component::outputs_;
  using Component::get_clock;
};

class ComponentTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    component_ = std::make_shared<ComponentPublicInterface>(rclcpp::NodeOptions());
  }

  std::shared_ptr<ComponentPublicInterface> component_;
};

TEST_F(ComponentTest, AddOutput) {
  std::shared_ptr<State> data = make_shared_state(CartesianState::Random("test"));
  component_->add_output("test", data, true);
  auto outputs_iterator = component_->outputs_.find("test");
  EXPECT_TRUE(outputs_iterator != component_->outputs_.end());
  EXPECT_NO_THROW(component_->outputs_.at("test")->publish());
}

} // namespace modulo_components