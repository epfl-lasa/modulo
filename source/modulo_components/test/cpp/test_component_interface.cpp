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
  }

  void SetUp() override {
    component_ = std::make_shared<ComponentInterface<rclcpp::Node, rclcpp::GenericPublisher>>(rclcpp::NodeOptions());
  }

  std::map<std::string, utilities::PredicateVariant> get_predicate_map() {
    return component_->predicates_;
  }

  void add_predicate(const std::string& name, bool value) {
    component_->add_predicate(name, value);
  }

  void add_predicate(const std::string& predicate_name, const std::function<bool(void)>& predicate_function) {
    component_->add_predicate(predicate_name, predicate_function);
  }

  bool get_predicate(const std::string& predicate_name) const {
    return component_->get_predicate(predicate_name);
  }

  void set_predicate(const std::string& predicate_name, bool predicate_value) {
    component_->set_predicate(predicate_name, predicate_value);
  }

  std::shared_ptr<ComponentInterface<rclcpp::Node, rclcpp::GenericPublisher>> component_;
};

TEST_F(ComponentInterfaceTest, AddBoolPredicate) {
  add_predicate("foo", true);
  auto predicates = get_predicate_map();
  auto predicate_iterator = predicates.find("foo");
  EXPECT_TRUE(predicate_iterator != predicates.end());
  auto value = std::get<bool>(predicate_iterator->second);
  EXPECT_TRUE(value);
}

TEST_F(ComponentInterfaceTest, AddFunctionPredicate) {
  add_predicate("bar", [&]() { return false; });
  auto predicates = get_predicate_map();
  auto predicate_iterator = predicates.find("bar");
  EXPECT_TRUE(predicate_iterator != predicates.end());
  auto value_callback = std::get<std::function<bool(void)>>(predicate_iterator->second);
  EXPECT_FALSE((value_callback)());
}

TEST_F(ComponentInterfaceTest, GetPredicateValue) {
  add_predicate("foo", true);
  EXPECT_TRUE(get_predicate("foo"));
  add_predicate("bar", [&]() { return false; });
  EXPECT_FALSE(get_predicate("bar"));
  EXPECT_THROW(get_predicate("test"), exceptions::PredicateNotFoundException);
}

TEST_F(ComponentInterfaceTest, SetPredicateValue) {
  add_predicate("foo", true);
  set_predicate("foo", false);
  EXPECT_FALSE(get_predicate("foo"));
  EXPECT_THROW(set_predicate("bar", true), exceptions::PredicateNotFoundException);
  add_predicate("bar", [&]() { return false; });
  set_predicate("bar", true);
  EXPECT_TRUE(get_predicate("bar"));
}

} // namespace modulo_components