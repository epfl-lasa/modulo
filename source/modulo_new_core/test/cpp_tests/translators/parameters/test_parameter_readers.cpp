#include <gtest/gtest.h>

#include <clproto.h>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/joint/JointState.hpp>
#include <state_representation/space/joint/JointPositions.hpp>

#include "modulo_new_core/translators/parameter_translators.hpp"

using namespace modulo_new_core::translators;

// Parameterized test fixture by type and expected value
// See also: http://www.ashermancinelli.com/gtest-type-val-param

template<typename T> using RParamT = std::vector<std::tuple<T, state_representation::ParameterType>>;

static std::tuple<
    RParamT<bool>,
    RParamT<std::vector<bool>>,
    RParamT<int>,
    RParamT<std::vector<int>>,
    RParamT<double>,
    RParamT<std::vector<double>>,
    RParamT<std::string>,
    RParamT<std::vector<std::string>>
> read_test_params{{
                       std::make_tuple(true, state_representation::ParameterType::BOOL),
                       std::make_tuple(false, state_representation::ParameterType::BOOL),
                   }, {
                       std::make_tuple(
                           std::vector<bool>({true, false, true}), state_representation::ParameterType::BOOL_ARRAY),
                   }, {
                       std::make_tuple(0, state_representation::ParameterType::INT),
                       std::make_tuple(1, state_representation::ParameterType::INT),
                   }, {
                       std::make_tuple(std::vector<int>({1, 2, 3}), state_representation::ParameterType::INT_ARRAY),
                   }, {
                       std::make_tuple(1.0, state_representation::ParameterType::DOUBLE),
                   }, {
                       std::make_tuple(
                           std::vector<double>({true, false, true}), state_representation::ParameterType::DOUBLE_ARRAY),
                   }, {
                       std::make_tuple("test", state_representation::ParameterType::STRING),
                   }, {
                       std::make_tuple(
                           std::vector<std::string>({"1", "2", "3"}),
                           state_representation::ParameterType::STRING_ARRAY),
                   },
};

template<typename T>
class ReadParameterTest : public testing::Test {
public:
  ReadParameterTest() : test_params_{std::get<RParamT<T>>(read_test_params)} {}
protected:
  void test_read_parameter(T value, state_representation::ParameterType expected_type) {
    ros_param_ = rclcpp::Parameter("test", value);
    param_ = read_parameter(ros_param_);
    ASSERT_NO_THROW(param_ = read_parameter(ros_param_));
    EXPECT_EQ(param_->get_name(), ros_param_.get_name());
    EXPECT_EQ(param_->get_parameter_type(), expected_type);
    EXPECT_EQ(param_->get_parameter_value<T>(), value);
  }

  void test_rewrite_parameter() {
    rclcpp::Parameter new_ros_param;
    ASSERT_NO_THROW(new_ros_param = write_parameter(param_));
    EXPECT_EQ(new_ros_param, ros_param_);
  }

  std::shared_ptr<state_representation::ParameterInterface> param_;
  rclcpp::Parameter ros_param_;
  RParamT<T> test_params_;
};
TYPED_TEST_SUITE_P(ReadParameterTest);

TYPED_TEST_P(ReadParameterTest, ReadAndReWrite) {
  for (auto const& [value, type]: this->test_params_) {
    this->test_read_parameter(value, type);
    this->test_rewrite_parameter();
  }
}

REGISTER_TYPED_TEST_SUITE_P(ReadParameterTest, ReadAndReWrite);

using ReadTestTypes = testing::Types<
    bool, std::vector<bool>, int, std::vector<int>, double, std::vector<double>, std::string, std::vector<std::string>>;
INSTANTIATE_TYPED_TEST_SUITE_P(TestPrefix, ReadParameterTest, ReadTestTypes);

/*
 * Test reading and rewriting parameters containing states
 */

static std::tuple<
    state_representation::CartesianState,
    state_representation::CartesianPose,
    state_representation::JointState,
    state_representation::JointPositions
> read_test_states{
    state_representation::CartesianState::Random("frame", "reference"),
    state_representation::CartesianPose::Random("frame", "reference"),
    state_representation::JointState::Random("robot", 3),
    state_representation::JointPositions::Random("robot", 3),
};

template<typename T>
class ReadStateParameterTest : public testing::Test {
public:
  ReadStateParameterTest() : test_state_{std::get<T>(read_test_states)} {}
protected:

  void check_state_equality(const T& new_state) {
    EXPECT_STREQ(new_state.get_name().c_str(), this->test_state_.get_name().c_str());
    EXPECT_EQ(new_state.get_type(), this->test_state_.get_type());
    EXPECT_NEAR(state_representation::dist(new_state, this->test_state_), 0.0, 1e-9);
  }

  void test_read_parameter() {
    std::string json = clproto::to_json<T>(this->test_state_);
    ros_param_ = rclcpp::Parameter("test", json);
    param_ = read_parameter(ros_param_);
    ASSERT_NO_THROW(param_ = read_parameter(ros_param_));
    EXPECT_EQ(param_->get_name(), ros_param_.get_name());
    EXPECT_EQ(param_->get_parameter_type(), state_representation::ParameterType::STATE);
    EXPECT_EQ(param_->get_parameter_state_type(), this->test_state_.get_type());

    ASSERT_NO_THROW(check_state_equality(param_->get_parameter_value<T>()));
  }

  void test_rewrite_parameter() {
    rclcpp::Parameter new_ros_param;
    ASSERT_NO_THROW(new_ros_param = write_parameter(param_));
    EXPECT_STREQ(new_ros_param.get_name().c_str(), ros_param_.get_name().c_str());
    EXPECT_EQ(new_ros_param.get_type(), ros_param_.get_type());
    EXPECT_NO_THROW(check_state_equality(clproto::from_json<T>(new_ros_param.as_string())));
  }

  std::shared_ptr<state_representation::ParameterInterface> param_;
  rclcpp::Parameter ros_param_;
  T test_state_;
};
TYPED_TEST_SUITE_P(ReadStateParameterTest);

TYPED_TEST_P(ReadStateParameterTest, ReadAndReWriteState) {
  this->test_read_parameter();
  this->test_rewrite_parameter();
}

REGISTER_TYPED_TEST_SUITE_P(ReadStateParameterTest, ReadAndReWriteState);

using ReadStateTestTypes = testing::Types<
    state_representation::CartesianState,
    state_representation::CartesianPose,
    state_representation::JointState,
    state_representation::JointPositions
>;
INSTANTIATE_TYPED_TEST_SUITE_P(TestPrefix, ReadStateParameterTest, ReadStateTestTypes);