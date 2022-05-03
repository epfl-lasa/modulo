#include <gtest/gtest.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/node_options.hpp>

#include <state_representation/exceptions/InvalidParameterException.hpp>

#include "modulo_components/ComponentInterface.hpp"
#include "modulo_new_core/EncodedState.hpp"

namespace modulo_components {

class ComponentInterfaceParameterPublicInterface : public ComponentInterface<rclcpp::Node> {
public:
  explicit ComponentInterfaceParameterPublicInterface(const rclcpp::NodeOptions& node_options) :
      ComponentInterface<rclcpp::Node>(node_options, modulo_new_core::communication::PublisherType::PUBLISHER) {}
  using ComponentInterface<rclcpp::Node>::add_parameter;
  using ComponentInterface<rclcpp::Node>::get_parameter;
  using ComponentInterface<rclcpp::Node>::get_parameter_value;
  using ComponentInterface<rclcpp::Node>::set_parameter_value;
  using ComponentInterface<rclcpp::Node>::parameter_map_;

  bool validate_parameter(const std::shared_ptr<state_representation::ParameterInterface>&) override {
    validate_was_called = true;
    return validation_return_value;
  }

  rclcpp::Parameter get_ros_parameter(const std::string& name) {
    return rclcpp::Node::get_parameter(name);
  }

  rcl_interfaces::msg::SetParametersResult set_ros_parameter(const rclcpp::Parameter& parameter) {
    return rclcpp::Node::set_parameter(parameter);
  }

  std::string get_parameter_description(const std::string& name) {
    return rclcpp::Node::describe_parameter(name).description;
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
    component_ = std::make_shared<ComponentInterfaceParameterPublicInterface>(rclcpp::NodeOptions());
    param_ = state_representation::make_shared_parameter("test", 1);
  }

  template<typename T>
  void expect_parameter_value(const T& value) {
    EXPECT_STREQ(component_->get_ros_parameter("test").value_to_string().c_str(), std::to_string(value).c_str());
    EXPECT_EQ(component_->get_parameter_value<T>("test"), value);
    EXPECT_EQ(component_->parameter_map_.get_parameter_value<T>("test"), value);
  }

  std::shared_ptr<ComponentInterfaceParameterPublicInterface> component_;
  std::shared_ptr<state_representation::Parameter<int>> param_;
};

TEST_F(ComponentInterfaceParameterTest, AddParameter) {
  EXPECT_THROW(auto discard = component_->get_parameter("test"),
               state_representation::exceptions::InvalidParameterException);
  EXPECT_THROW(component_->get_ros_parameter("test"), rclcpp::exceptions::ParameterNotDeclaredException);
  component_->add_parameter(param_, "Test parameter");

  // Adding the parameter should declare and set the value and call the validation function
  EXPECT_TRUE(component_->validate_was_called);
  EXPECT_NO_THROW(auto discard = component_->get_parameter("test"));
  EXPECT_NO_THROW(component_->get_ros_parameter("test"));
  expect_parameter_value<int>(1);
}

TEST_F(ComponentInterfaceParameterTest, AddNameValueParameter) {
  EXPECT_THROW(auto discard = component_->get_parameter("test"),
               state_representation::exceptions::InvalidParameterException);
  EXPECT_THROW(component_->get_ros_parameter("test"), rclcpp::exceptions::ParameterNotDeclaredException);
  component_->add_parameter("test", 1, "Test parameter");

  // Adding the parameter should declare and set the value and call the validation function
  EXPECT_TRUE(component_->validate_was_called);
  EXPECT_NO_THROW(auto discard = component_->get_parameter("test"));
  EXPECT_NO_THROW(component_->get_ros_parameter("test"));
  expect_parameter_value<int>(1);
}

TEST_F(ComponentInterfaceParameterTest, AddParameterAgain) {
  EXPECT_THROW(auto
                   discard = component_->get_parameter("test"),
               state_representation::exceptions::InvalidParameterException);
  EXPECT_THROW(component_->get_ros_parameter("test"), rclcpp::exceptions::ParameterNotDeclaredException);
  component_->add_parameter(param_, "Test parameter");

  // Adding an existing parameter again should just set the value
  component_->validate_was_called = false;
  EXPECT_NO_THROW(component_->add_parameter("test", 2, "foo"));
  EXPECT_TRUE(component_->validate_was_called);
  expect_parameter_value<int>(2);
  EXPECT_EQ(param_->get_value(), 2);
}

TEST_F(ComponentInterfaceParameterTest, SetParameter) {
  // setting before adding should not work
  EXPECT_THROW(component_->set_parameter_value("test", 1), rclcpp::exceptions::ParameterNotDeclaredException);

  // validation should not be invoked as the parameter did not exist
  EXPECT_FALSE(component_->validate_was_called);
  EXPECT_THROW(component_->get_ros_parameter("test"), rclcpp::exceptions::ParameterNotDeclaredException);

  component_->add_parameter(param_, "Test parameter");

  // Setting the parameter value should call the validation function and update the referenced value
  component_->validate_was_called = false;
  EXPECT_NO_THROW(component_->set_parameter_value("test", 2));
  EXPECT_TRUE(component_->validate_was_called);
  expect_parameter_value<int>(2);
  EXPECT_EQ(param_->get_value(), 2);

  // If the validation function returns false, setting the parameter value should _not_ update the referenced value
  component_->validate_was_called = false;
  component_->validation_return_value = false;
  EXPECT_THROW(component_->set_parameter_value("test", 3), state_representation::exceptions::InvalidParameterException);
  EXPECT_TRUE(component_->validate_was_called);
  expect_parameter_value<int>(2);
  EXPECT_EQ(param_->get_value(), 2);

  // Setting a value with an incompatible type should not update the parameter
  EXPECT_THROW(component_->set_parameter_value<std::string>("test", "foo"),
               state_representation::exceptions::InvalidParameterException);
  EXPECT_TRUE(component_->validate_was_called);
  expect_parameter_value<int>(2);
  EXPECT_EQ(param_->get_value(), 2);
}

TEST_F(ComponentInterfaceParameterTest, SetParameterROS) {
  component_->add_parameter(param_, "Test parameter");

  // Setting the parameter value should call the validation function and update the referenced value
  component_->validate_was_called = false;
  auto result = component_->set_ros_parameter({"test", 2});
  EXPECT_TRUE(component_->validate_was_called);
  EXPECT_TRUE(result.successful);
  expect_parameter_value<int>(2);
  EXPECT_EQ(param_->get_value(), 2);

  // If the validation function returns false, setting the parameter value should _not_ update the referenced value
  component_->validate_was_called = false;
  component_->validation_return_value = false;
  result = component_->set_ros_parameter({"test", 3});
  EXPECT_TRUE(component_->validate_was_called);
  EXPECT_FALSE(result.successful);
  expect_parameter_value<int>(2);
  EXPECT_EQ(param_->get_value(), 2);
}

TEST_F(ComponentInterfaceParameterTest, GetParameterDescription) {
  component_->add_parameter(param_, "Test parameter");
  EXPECT_STREQ(component_->get_parameter_description("test").c_str(), "Test parameter");

  EXPECT_THROW(component_->get_parameter_description("foo"), rclcpp::exceptions::ParameterNotDeclaredException);
}

TEST_F(ComponentInterfaceParameterTest, ReadOnlyParameter) {
  // Adding a read-only parameter should behave normally
  component_->add_parameter(param_, "Test parameter", true);
  EXPECT_TRUE(component_->validate_was_called);
  EXPECT_NO_THROW(auto discard = component_->get_parameter("test"));
  EXPECT_NO_THROW(component_->get_ros_parameter("test"));
  expect_parameter_value<int>(1);

  // Trying to set the value of the read-only parameter should fail before the validation step
  component_->validate_was_called = false;
  EXPECT_THROW(component_->set_parameter_value("test", 2), state_representation::exceptions::InvalidParameterException);
  EXPECT_FALSE(component_->validate_was_called);
  expect_parameter_value<int>(1);
  EXPECT_EQ(param_->get_value(), 1);

  // Setting the value on the ROS interface should also fail
  component_->validate_was_called = false;
  auto result = component_->set_ros_parameter({"test", 2});
  EXPECT_FALSE(component_->validate_was_called);
  EXPECT_FALSE(result.successful);
  expect_parameter_value<int>(1);
  EXPECT_EQ(param_->get_value(), 1);
}

} // namespace modulo_components