#include <gtest/gtest.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/node_options.hpp>

#include "modulo_components/ComponentInterface.hpp"
#include "modulo_new_core/EncodedState.hpp"

namespace modulo_components {
class ComponentInterfaceTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    component_ = std::make_shared<ComponentInterface<rclcpp::Node>>(
        rclcpp::NodeOptions(), modulo_new_core::communication::PublisherType::PUBLISHER
    );
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

  void set_predicate(const std::string& predicate_name, const std::function<bool(void)>& predicate_function) {
    component_->set_predicate(predicate_name, predicate_function);
  }

  std::shared_ptr<ComponentInterface<rclcpp::Node>> component_;
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
  // predicate does not exist, expect false
  EXPECT_FALSE(get_predicate("test"));
  // error in callback function except false
  add_predicate(
      "error", [&]() {
        throw std::runtime_error("An error occurred");
        return false;
      }
  );
  EXPECT_FALSE(get_predicate("error"));
}

TEST_F(ComponentInterfaceTest, SetPredicateValue) {
  add_predicate("foo", true);
  set_predicate("foo", false);
  EXPECT_FALSE(get_predicate("foo"));
  // predicate does not exist but set it anyway
  set_predicate("bar", true);
  EXPECT_TRUE(get_predicate("bar"));
  set_predicate("bar", [&]() { return false; });
  EXPECT_FALSE(get_predicate("bar"));
}

} // namespace modulo_components