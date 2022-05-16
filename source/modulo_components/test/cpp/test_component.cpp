#include <gtest/gtest.h>
#include <rclcpp/publisher.hpp>
#include <rclcpp/node_options.hpp>

#include "modulo_components/Component.hpp"
#include "modulo_new_core/EncodedState.hpp"

using namespace state_representation;

namespace modulo_components {

class ComponentPublicInterface : public Component {
public:
  explicit ComponentPublicInterface(const rclcpp::NodeOptions& node_options, bool start_thread = true) :
      Component(node_options, start_thread) {}
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
    component_ = std::make_shared<ComponentPublicInterface>(rclcpp::NodeOptions(), false);
  }

  std::shared_ptr<ComponentPublicInterface> component_;
};

TEST_F(ComponentTest, AddOutput) {
  std::shared_ptr<State> data = make_shared_state(CartesianState::Random("test"));
  component_->add_output("_tEsT_#1@3", data, true);
  EXPECT_TRUE(component_->outputs_.find("test_13") != component_->outputs_.end());
  EXPECT_NO_THROW(component_->outputs_.at("test_13")->publish());

  auto new_data = std::make_shared<bool>(false);
  component_->add_output("test_13", new_data, true);
  EXPECT_EQ(component_->outputs_.at("test_13")->get_message_pair()->get_type(),
            modulo_new_core::communication::MessageType::ENCODED_STATE);
}

} // namespace modulo_components