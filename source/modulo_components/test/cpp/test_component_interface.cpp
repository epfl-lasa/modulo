#include <gtest/gtest.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/node_options.hpp>

#include "modulo_components/ComponentInterface.h"
#include "modulo_new_core/EncodedState.hpp"

namespace modulo_components {
class ComponentInterfaceTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
    if (component_ == nullptr) {
      component_ = std::make_unique<ComponentInterface<rclcpp::Node, rclcpp::GenericPublisher>>(rclcpp::NodeOptions());
    }
  }

  void TearDown() override {
    component_->predicates_.clear();
  }

  static std::unique_ptr<ComponentInterface<rclcpp::Node, rclcpp::GenericPublisher>> component_;
};

std::unique_ptr<ComponentInterface<rclcpp::Node, rclcpp::GenericPublisher>> ComponentInterfaceTest::component_ = nullptr;

TEST_F(ComponentInterfaceTest, AddBoolPredicate) {
  component_->add_predicate("foo", true);
  auto predicate_iterator = component_->predicates_.find("foo");
  EXPECT_TRUE(predicate_iterator != component_->predicates_.end());
  auto value = std::get<bool>(predicate_iterator->second);
  EXPECT_TRUE(value);
}

TEST_F(ComponentInterfaceTest, AddFunctionPredicate) {
  component_->add_predicate("bar", [&]() { return false; });
  auto predicate_iterator = component_->predicates_.find("bar");
  EXPECT_TRUE(predicate_iterator != component_->predicates_.end());
  auto value_callback = std::get<std::function<bool(void)>>(predicate_iterator->second);
  EXPECT_FALSE((value_callback)());
}

TEST_F(ComponentInterfaceTest, GetPredicateValue) {
  component_->add_predicate("foo", true);
  EXPECT_TRUE(component_->get_predicate("foo"));
  component_->add_predicate("bar", [&]() { return false; });
  EXPECT_FALSE(component_->get_predicate("bar"));
  // TODO suppress the no discard warning
  EXPECT_THROW(component_->get_predicate("test"), exceptions::PredicateNotFoundException);
}

TEST_F(ComponentInterfaceTest, SetPredicateValue) {
  component_->add_predicate("foo", true);
  component_->set_predicate("foo", false);
  EXPECT_FALSE(component_->get_predicate("foo"));
  EXPECT_THROW(component_->set_predicate("bar", true), exceptions::PredicateNotFoundException);
  component_->add_predicate("bar", [&]() { return false; });
  component_->set_predicate("bar", true);
  EXPECT_TRUE(component_->get_predicate("bar"));
}

} // namespace modulo_components