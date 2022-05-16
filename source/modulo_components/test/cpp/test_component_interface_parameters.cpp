#include <gtest/gtest.h>

#include <state_representation/exceptions/InvalidParameterException.hpp>

#include "modulo_components/ComponentInterface.hpp"
#include "modulo_new_core/EncodedState.hpp"

namespace modulo_components {

template<class NodeT>
class ComponentInterfaceParameterPublicInterface : public ComponentInterface<NodeT> {
public:
  explicit ComponentInterfaceParameterPublicInterface(
      const rclcpp::NodeOptions& node_options, modulo_new_core::communication::PublisherType publisher_type
  ) : ComponentInterface<NodeT>(node_options, publisher_type) {}
  using ComponentInterface<NodeT>::add_parameter;
  using ComponentInterface<NodeT>::get_parameter;
  using ComponentInterface<NodeT>::get_parameter_value;
  using ComponentInterface<NodeT>::set_parameter_value;
  using ComponentInterface<NodeT>::parameter_map_;

  bool validate_parameter(const std::shared_ptr<state_representation::ParameterInterface>&) override {
    validate_was_called = true;
    return validation_return_value;
  }

  rclcpp::Parameter get_ros_parameter(const std::string& name) {
    return NodeT::get_parameter(name);
  }

  rcl_interfaces::msg::SetParametersResult set_ros_parameter(const rclcpp::Parameter& parameter) {
    return NodeT::set_parameter(parameter);
  }

  std::string get_parameter_description(const std::string& name) {
    return NodeT::describe_parameter(name).description;
  }

  bool validate_was_called = false;
  bool validation_return_value = true;
};

template<class NodeT>
class ComponentInterfaceParameterTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    if (std::is_same<NodeT, rclcpp::Node>::value) {
      this->component_ = std::make_shared<ComponentInterfaceParameterPublicInterface<NodeT>>(
          rclcpp::NodeOptions(), modulo_new_core::communication::PublisherType::PUBLISHER
      );
    } else if (std::is_same<NodeT, rclcpp_lifecycle::LifecycleNode>::value) {
      this->component_ = std::make_shared<ComponentInterfaceParameterPublicInterface<NodeT>>(
          rclcpp::NodeOptions(), modulo_new_core::communication::PublisherType::LIFECYCLE_PUBLISHER
      );
    }
    param_ = state_representation::make_shared_parameter("test", 1);
  }

  template<typename T>
  void expect_parameter_value(const T& value) {
    EXPECT_STREQ(this->component_->get_ros_parameter("test").value_to_string().c_str(), std::to_string(value).c_str());
    EXPECT_EQ(this->component_->template get_parameter_value<T>("test"), value);
    EXPECT_EQ(this->component_->parameter_map_.template get_parameter_value<T>("test"), value);
  }

  std::shared_ptr<ComponentInterfaceParameterPublicInterface<NodeT>> component_;
  std::shared_ptr<state_representation::Parameter<int>> param_;
};
using NodeTypes = ::testing::Types<rclcpp::Node, rclcpp_lifecycle::LifecycleNode>;
TYPED_TEST_SUITE(ComponentInterfaceParameterTest, NodeTypes);

TYPED_TEST(ComponentInterfaceParameterTest, AddParameter) {
  EXPECT_THROW(auto discard = this->component_->get_parameter("test"),
               state_representation::exceptions::InvalidParameterException);
  EXPECT_THROW(this->component_->get_ros_parameter("test"), rclcpp::exceptions::ParameterNotDeclaredException);
  this->component_->add_parameter(this->param_, "Test parameter");

  // Adding the parameter should declare and set the value and call the validation function
  EXPECT_TRUE(this->component_->validate_was_called);
  EXPECT_NO_THROW(auto discard = this->component_->get_parameter("test"));
  EXPECT_NO_THROW(this->component_->get_ros_parameter("test"));
  this->template expect_parameter_value<int>(1);
}

TYPED_TEST(ComponentInterfaceParameterTest, AddNameValueParameter) {
  EXPECT_THROW(auto discard = this->component_->get_parameter("test"),
               state_representation::exceptions::InvalidParameterException);
  EXPECT_THROW(this->component_->get_ros_parameter("test"), rclcpp::exceptions::ParameterNotDeclaredException);
  this->component_->add_parameter("test", 1, "Test parameter");

  // Adding the parameter should declare and set the value and call the validation function
  EXPECT_TRUE(this->component_->validate_was_called);
  EXPECT_NO_THROW(auto discard = this->component_->get_parameter("test"));
  EXPECT_NO_THROW(this->component_->get_ros_parameter("test"));
  this->template expect_parameter_value<int>(1);
}

TYPED_TEST(ComponentInterfaceParameterTest, AddParameterAgain) {
  EXPECT_THROW(auto
                   discard = this->component_->get_parameter("test"),
               state_representation::exceptions::InvalidParameterException);
  EXPECT_THROW(this->component_->get_ros_parameter("test"), rclcpp::exceptions::ParameterNotDeclaredException);
  this->component_->add_parameter(this->param_, "Test parameter");

  // Adding an existing parameter again should just set the value
  this->component_->validate_was_called = false;
  EXPECT_NO_THROW(this->component_->add_parameter("test", 2, "foo"));
  EXPECT_TRUE(this->component_->validate_was_called);
  this->template expect_parameter_value<int>(2);
  EXPECT_EQ(this->param_->get_value(), 2);
}

TYPED_TEST(ComponentInterfaceParameterTest, SetParameter) {
  // setting before adding should not work
  EXPECT_THROW(this->component_->set_parameter_value("test", 1), rclcpp::exceptions::ParameterNotDeclaredException);

  // validation should not be invoked as the parameter did not exist
  EXPECT_FALSE(this->component_->validate_was_called);
  EXPECT_THROW(this->component_->get_ros_parameter("test"), rclcpp::exceptions::ParameterNotDeclaredException);

  this->component_->add_parameter(this->param_, "Test parameter");

  // Setting the parameter value should call the validation function and update the referenced value
  this->component_->validate_was_called = false;
  EXPECT_NO_THROW(this->component_->set_parameter_value("test", 2));
  EXPECT_TRUE(this->component_->validate_was_called);
  this->template expect_parameter_value<int>(2);
  EXPECT_EQ(this->param_->get_value(), 2);

  // If the validation function returns false, setting the parameter value should _not_ update the referenced value
  this->component_->validate_was_called = false;
  this->component_->validation_return_value = false;
  EXPECT_THROW(this->component_->set_parameter_value("test", 3),
               state_representation::exceptions::InvalidParameterException);
  EXPECT_TRUE(this->component_->validate_was_called);
  this->template expect_parameter_value<int>(2);
  EXPECT_EQ(this->param_->get_value(), 2);

  // Setting a value with an incompatible type should not update the parameter
  EXPECT_THROW(this->component_->template set_parameter_value<std::string>("test", "foo"),
               state_representation::exceptions::InvalidParameterException);
  EXPECT_TRUE(this->component_->validate_was_called);
  this->template expect_parameter_value<int>(2);
  EXPECT_EQ(this->param_->get_value(), 2);
}

TYPED_TEST(ComponentInterfaceParameterTest, SetParameterROS) {
  this->component_->add_parameter(this->param_, "Test parameter");

  // Setting the parameter value should call the validation function and update the referenced value
  this->component_->validate_was_called = false;
  auto result = this->component_->set_ros_parameter({"test", 2});
  EXPECT_TRUE(this->component_->validate_was_called);
  EXPECT_TRUE(result.successful);
  this->template expect_parameter_value<int>(2);
  EXPECT_EQ(this->param_->get_value(), 2);

  // If the validation function returns false, setting the parameter value should _not_ update the referenced value
  this->component_->validate_was_called = false;
  this->component_->validation_return_value = false;
  result = this->component_->set_ros_parameter({"test", 3});
  EXPECT_TRUE(this->component_->validate_was_called);
  EXPECT_FALSE(result.successful);
  this->template expect_parameter_value<int>(2);
  EXPECT_EQ(this->param_->get_value(), 2);
}

TYPED_TEST(ComponentInterfaceParameterTest, GetParameterDescription) {
  this->component_->add_parameter(this->param_, "Test parameter");
  EXPECT_STREQ(this->component_->get_parameter_description("test").c_str(), "Test parameter");

  EXPECT_THROW(this->component_->get_parameter_description("foo"), rclcpp::exceptions::ParameterNotDeclaredException);
}

TYPED_TEST(ComponentInterfaceParameterTest, ReadOnlyParameter) {
  // Adding a read-only parameter should behave normally
  this->component_->add_parameter(this->param_, "Test parameter", true);
  EXPECT_TRUE(this->component_->validate_was_called);
  EXPECT_NO_THROW(auto discard = this->component_->get_parameter("test"));
  EXPECT_NO_THROW(this->component_->get_ros_parameter("test"));
  this->template expect_parameter_value<int>(1);

  // Trying to set the value of the read-only parameter should fail before the validation step
  this->component_->validate_was_called = false;
  EXPECT_THROW(this->component_->set_parameter_value("test", 2),
               state_representation::exceptions::InvalidParameterException);
  EXPECT_FALSE(this->component_->validate_was_called);
  this->template expect_parameter_value<int>(1);
  EXPECT_EQ(this->param_->get_value(), 1);

  // Setting the value on the ROS interface should also fail
  this->component_->validate_was_called = false;
  auto result = this->component_->set_ros_parameter({"test", 2});
  EXPECT_FALSE(this->component_->validate_was_called);
  EXPECT_FALSE(result.successful);
  this->template expect_parameter_value<int>(1);
  EXPECT_EQ(this->param_->get_value(), 1);
}

} // namespace modulo_components