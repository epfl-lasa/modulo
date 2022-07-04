#include <gtest/gtest.h>

#include "modulo_components/exceptions/ComponentParameterException.hpp"
#include "modulo_core/EncodedState.hpp"
#include "test_modulo_components/component_public_interfaces.hpp"

namespace modulo_components {

template<class NodeT>
class EmtpyParameterInterface : public ComponentInterfacePublicInterface<NodeT> {
public:
  explicit EmtpyParameterInterface(
      const rclcpp::NodeOptions& node_options, modulo_core::communication::PublisherType publisher_type
  ) : ComponentInterfacePublicInterface<NodeT>(node_options, publisher_type, "EmtpyParameterInterface") {
    this->add_parameter(std::make_shared<Parameter<std::string>>("name"), "Test parameter");
  };

private:
  bool validate_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override {
    if (parameter->get_name() == "name") {
      if (parameter->get_parameter_value<std::string>().empty()) {
        RCLCPP_ERROR(this->get_logger(), "Provide a non empty value for parameter 'name'");
        return false;
      }
    }
    return true;
  }
};

template<class NodeT>
class ComponentInterfaceEmptyParameterTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    if (std::is_same<NodeT, rclcpp::Node>::value) {
      this->component_ = std::make_shared<EmtpyParameterInterface<NodeT>>(
          rclcpp::NodeOptions(), modulo_core::communication::PublisherType::PUBLISHER
      );
    } else if (std::is_same<NodeT, rclcpp_lifecycle::LifecycleNode>::value) {
      this->component_ = std::make_shared<EmtpyParameterInterface<NodeT>>(
          rclcpp::NodeOptions(), modulo_core::communication::PublisherType::LIFECYCLE_PUBLISHER
      );
    }
  }

  std::shared_ptr<EmtpyParameterInterface<NodeT>> component_;
};
using NodeTypes = ::testing::Types<rclcpp::Node, rclcpp_lifecycle::LifecycleNode>;
TYPED_TEST_SUITE(ComponentInterfaceEmptyParameterTest, NodeTypes);

TYPED_TEST(ComponentInterfaceEmptyParameterTest, ValidateEmptyParameter) {
  // component comes with empty parameter 'name'
  EXPECT_EQ(this->component_->get_parameter("name")->get_parameter_type(), ParameterType::STRING);
  EXPECT_TRUE(this->component_->get_parameter("name")->is_empty());
  auto ros_param = rclcpp::Parameter();
  EXPECT_THROW(ros_param = this->component_->get_ros_parameter("name"),
               rclcpp::exceptions::ParameterUninitializedException);
//  EXPECT_NO_THROW(this->component_->get_ros_parameter("name"););
//  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_NOT_SET);

  // Trying to overwrite with an empty parameter is not allowed
  this->component_->add_parameter(std::make_shared<Parameter<bool>>("name"), "Test parameter");
  EXPECT_EQ(this->component_->get_parameter("name")->get_parameter_type(), ParameterType::STRING);
  EXPECT_THROW(ros_param = this->component_->get_ros_parameter("name"),
               rclcpp::exceptions::ParameterUninitializedException);
//  EXPECT_NO_THROW(ros_param = this->component_->get_ros_parameter("name"););
//  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_NOT_SET);

  // Set parameter with empty parameter isn't possible because there is no method for that
  // component->set_parameter(std::make_shared<Parameter<std::string>>("name"));

  // Set parameter value from rclcpp interface should update ROS parameter type
  this->component_->set_ros_parameter({"name", "test"});
  EXPECT_EQ(this->component_->get_parameter("name")->get_parameter_type(), ParameterType::STRING);
  EXPECT_EQ(this->component_->get_parameter("name")->template get_parameter_value<std::string>(), "test");
  EXPECT_NO_THROW(ros_param = this->component_->get_ros_parameter("name"););
  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_STRING);
  EXPECT_EQ(ros_param.template get_value<std::string>(), "test");

  // Set parameter value from component interface
  this->component_->template set_parameter_value<std::string>("name", "again");
  EXPECT_EQ(this->component_->get_parameter("name")->get_parameter_type(), ParameterType::STRING);
  EXPECT_EQ(this->component_->get_parameter("name")->template get_parameter_value<std::string>(), "again");
  EXPECT_NO_THROW(ros_param = this->component_->get_ros_parameter("name"););
  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_STRING);
  EXPECT_EQ(ros_param.template get_value<std::string>(), "again");

  // Setting it with empty value should be rejected in parameter evaluation
  this->component_->template set_parameter_value<std::string>("name", "");
  EXPECT_EQ(this->component_->get_parameter("name")->get_parameter_type(), ParameterType::STRING);
  EXPECT_EQ(this->component_->get_parameter("name")->template get_parameter_value<std::string>(), "again");
  EXPECT_NO_THROW(ros_param = this->component_->get_ros_parameter("name"););
  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_STRING);
  EXPECT_EQ(ros_param.template get_value<std::string>(), "again");

  // Setting a parameter with type NOT_SET, undeclares that parameter, hence the last assertion fails!
  auto result = this->component_->set_ros_parameter(rclcpp::Parameter("name"));
  RCLCPP_ERROR_STREAM(this->component_->get_logger(), result.reason);
  EXPECT_EQ(this->component_->get_parameter("name")->get_parameter_type(), ParameterType::STRING);
  EXPECT_NO_THROW(ros_param = this->component_->get_ros_parameter("name"););
  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_NOT_SET);
//  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_STRING);
//  EXPECT_EQ(ros_param.get_value<std::string>(), "again");
}
} // namespace modulo_components
