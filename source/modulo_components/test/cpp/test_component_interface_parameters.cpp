#include <gtest/gtest.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/node_options.hpp>

#include <state_representation/exceptions/InvalidParameterException.hpp>

#include "modulo_components/ComponentInterface.hpp"
#include "modulo_new_core/EncodedState.hpp"

namespace modulo_components {

class ComponentInterfacePublicInterface : public ComponentInterface<rclcpp::Node> {
public:
  ComponentInterfacePublicInterface(const rclcpp::NodeOptions& node_options) :
      ComponentInterface<rclcpp::Node>(node_options, modulo_new_core::communication::PublisherType::PUBLISHER) {}
  using ComponentInterface<rclcpp::Node>::add_parameter;
  using ComponentInterface<rclcpp::Node>::get_parameter;
  using ComponentInterface<rclcpp::Node>::get_parameter_value;
  using ComponentInterface<rclcpp::Node>::parameter_map_;

  bool validate_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override {
    validate_was_called = true;
    return validation_return_value;
  }

  rclcpp::Parameter get_ros_parameter(const std::string& name) {
    return rclcpp::Node::get_parameter(name);
  }

  rcl_interfaces::msg::SetParametersResult set_ros_parameter(const rclcpp::Parameter& parameter) {
    return rclcpp::Node::set_parameter(parameter);
  }

  bool validate_was_called = false;
  bool validation_return_value = true;
};

class ComponentInterfaceParameterTest : public ::testing::Test {
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

  template<typename T>
  void expect_parameter_value(const T& value) {
    EXPECT_STREQ(component_->get_ros_parameter("test").value_to_string().c_str(), std::to_string(value).c_str());
    EXPECT_EQ(component_->get_parameter_value<T>("test"), value);
    EXPECT_EQ(component_->parameter_map_.get_parameter_value<T>("test"), value);
  }

  std::shared_ptr<ComponentInterfacePublicInterface> component_;
};


TEST_F(ComponentInterfaceParameterTest, AddParameter) {
  EXPECT_THROW(component_->get_parameter("test"), state_representation::exceptions::InvalidParameterException);
  EXPECT_THROW(component_->get_ros_parameter("test"), rclcpp::exceptions::ParameterNotDeclaredException);
  auto param = state_representation::make_shared_parameter("test", 1);
  component_->add_parameter(param);

  // Adding the parameter should declare and set the value and call the validation function
  EXPECT_TRUE(component_->validate_was_called);
  EXPECT_NO_THROW(component_->get_parameter("test"));
  EXPECT_NO_THROW(component_->get_ros_parameter("test"));
  expect_parameter_value<int>(1);

  // Setting the parameter value should call the validation function and update the referenced value
  component_->validate_was_called = false;
  component_->set_ros_parameter({"test", 2});
  EXPECT_TRUE(component_->validate_was_called);
  expect_parameter_value<int>(2);
  EXPECT_EQ(param->get_value(), 2);

  // If the validation function returns false, setting the parameter value should _not_ update the referenced value
  component_->validate_was_called = false;
  component_->validation_return_value = false;
  component_->set_ros_parameter({"test", 3});
  EXPECT_TRUE(component_->validate_was_called);
  expect_parameter_value<int>(2);
  EXPECT_EQ(param->get_value(), 2);
}

} // namespace modulo_components