#include <gtest/gtest.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/node_options.hpp>

#include "modulo_components/ComponentInterface.h"
#include "modulo_new_core/EncodedState.hpp"

namespace modulo_components {
class TestComponentInterface : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
    if (component_ == nullptr) {
      component_ =
          new ComponentInterface<rclcpp::Node, rclcpp::Publisher<modulo_new_core::EncodedState>>(rclcpp::NodeOptions());
    }
  }

  void TearDown() override {
    component_->predicates_.clear();
  }

  static ComponentInterface<rclcpp::Node, rclcpp::Publisher<modulo_new_core::EncodedState>>* component_;
};

ComponentInterface<rclcpp::Node, rclcpp::Publisher<modulo_new_core::EncodedState>>
    * TestComponentInterface::component_ = nullptr;

TEST_F(TestComponentInterface, test_add_bool_predicate) {
  component_->add_predicate("foo", true);
  auto predicate_iterator = component_->predicates_.find("foo");
  EXPECT_TRUE(predicate_iterator != component_->predicates_.end());
  auto value = std::get<bool>(predicate_iterator->second);
  EXPECT_TRUE(value);
}

TEST_F(TestComponentInterface, test_add_function_predicate) {
  component_->add_predicate("bar", [&]() { return false; });
  auto predicate_iterator = component_->predicates_.find("bar");
  EXPECT_TRUE(predicate_iterator != component_->predicates_.end());
  auto value = std::get<std::function<bool(void)>>(predicate_iterator->second);
  EXPECT_FALSE((value)());
}

TEST_F(TestComponentInterface, test_get_predicate_value) {
  component_->add_predicate("foo", true);
  EXPECT_TRUE(component_->get_predicate("foo"));
  component_->add_predicate("bar", [&]() { return false; });
  EXPECT_FALSE(component_->get_predicate("bar"));
  // TODO suppress the no discard warning
  EXPECT_THROW(component_->get_predicate("test"), exceptions::PredicateNotFoundException);
}

TEST_F(TestComponentInterface, test_set_predicate_value) {
  component_->add_predicate("foo", true);
  component_->set_predicate("foo", false);
  EXPECT_FALSE(component_->get_predicate("foo"));
  EXPECT_THROW(component_->set_predicate("bar", true), exceptions::PredicateNotFoundException);
  component_->add_predicate("bar", [&]() { return false; });
  component_->set_predicate("bar", true);
  EXPECT_TRUE(component_->get_predicate("bar"));
}

} // namespace modulo_components