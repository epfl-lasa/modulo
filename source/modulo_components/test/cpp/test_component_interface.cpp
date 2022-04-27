#include <gtest/gtest.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/node_options.hpp>

#include "modulo_components/ComponentInterface.hpp"
#include "modulo_new_core/EncodedState.hpp"

namespace modulo_components {

class ComponentInterfacePublicInterface : public ComponentInterface<rclcpp::Node> {
public:
  explicit ComponentInterfacePublicInterface(const rclcpp::NodeOptions& node_options) :
      ComponentInterface<rclcpp::Node>(node_options, modulo_new_core::communication::PublisherType::PUBLISHER) {}
  using ComponentInterface<rclcpp::Node>::add_predicate;
  using ComponentInterface<rclcpp::Node>::get_predicate;
  using ComponentInterface<rclcpp::Node>::set_predicate;
  using ComponentInterface<rclcpp::Node>::predicates_;
  using ComponentInterface<rclcpp::Node>::add_input;
  using ComponentInterface<rclcpp::Node>::inputs_;
};

class ComponentInterfaceTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    component_ = std::make_shared<ComponentInterfacePublicInterface>(rclcpp::NodeOptions());
  }

  std::shared_ptr<ComponentInterfacePublicInterface> component_;
};

TEST_F(ComponentInterfaceTest, AddBoolPredicate) {
  this->component_->add_predicate("foo", true);
  auto predicate_iterator = this->component_->predicates_.find("foo");
  EXPECT_TRUE(predicate_iterator != this->component_->predicates_.end());
  auto value = std::get<bool>(predicate_iterator->second);
  EXPECT_TRUE(value);
}

TEST_F(ComponentInterfaceTest, AddFunctionPredicate) {
  this->component_->add_predicate("bar", [&]() { return false; });
  auto predicate_iterator = component_->predicates_.find("bar");
  EXPECT_TRUE(predicate_iterator != component_->predicates_.end());
  auto value_callback = std::get<std::function<bool(void)>>(predicate_iterator->second);
  EXPECT_FALSE((value_callback)());
}

TEST_F(ComponentInterfaceTest, GetPredicateValue) {
  this->component_->add_predicate("foo", true);
  EXPECT_TRUE(this->component_->get_predicate("foo"));
  this->component_->add_predicate("bar", [&]() { return true; });
  EXPECT_TRUE(this->component_->get_predicate("bar"));
  // predicate does not exist, expect false
  EXPECT_FALSE(this->component_->get_predicate("test"));
  // error in callback function except false
  this->component_->add_predicate(
      "error", [&]() {
        throw std::runtime_error("An error occurred");
        return false;
      }
  );
  EXPECT_FALSE(this->component_->get_predicate("error"));
}

TEST_F(ComponentInterfaceTest, SetPredicateValue) {
  this->component_->add_predicate("foo", true);
  this->component_->set_predicate("foo", false);
  EXPECT_FALSE(this->component_->get_predicate("foo"));
  // predicate does not exist so setting won't do anything
  this->component_->set_predicate("bar", true);
  EXPECT_FALSE(this->component_->get_predicate("bar"));
  this->component_->add_predicate("bar", true);
  this->component_->set_predicate("bar", [&]() { return false; });
  EXPECT_FALSE(this->component_->get_predicate("bar"));
}

TEST_F(ComponentInterfaceTest, AddInput) {
  auto data = std::make_shared<bool>(true);
  EXPECT_NO_THROW(this->component_->add_input("_tEsT_#1@3", data, true));
  auto inputs_iterator = component_->inputs_.find("test_13");
  EXPECT_TRUE(inputs_iterator != component_->inputs_.end());

  EXPECT_NO_THROW(this->component_->template add_input<std_msgs::msg::Bool>(
      "_tEsT_#1@5", [](const std::shared_ptr<std_msgs::msg::Bool> msg) {}
  ));
  inputs_iterator = component_->inputs_.find("test_15");
  EXPECT_TRUE(inputs_iterator != component_->inputs_.end());
}
}