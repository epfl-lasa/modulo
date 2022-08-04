#include <gtest/gtest.h>

#include "test_modulo_core/communication_nodes.hpp"

using namespace modulo_core::communication;
using namespace state_representation;

class EncodedParameterCommunicationTest : public CommunicationTest {
protected:
  template<typename PubT, typename RecvT>
  void communicate(
      PubT publish_state, RecvT receive_state, bool expect_translation = true,
      std::function<void(const std::shared_ptr<PubT>&, const std::shared_ptr<RecvT>&)> test_function = {}
  ) {
    auto expected_type = receive_state.get_type();
    auto expected_parameter_type = receive_state.get_parameter_type();
    auto expected_name = expect_translation ? publish_state.get_name() : receive_state.get_name();
    auto expected_empty = expect_translation ? publish_state.is_empty() : receive_state.is_empty();
    auto pub_state = std::make_shared<PubT>(publish_state);
    auto recv_state = std::make_shared<RecvT>(receive_state);
    auto pub_message = make_shared_message_pair(pub_state, this->clock_);
    auto recv_message = make_shared_message_pair(recv_state, this->clock_);
    this->add_nodes<modulo_core::EncodedState>("/test_topic", pub_message, recv_message);
    this->exec_->template spin_until_future_complete(
        std::dynamic_pointer_cast<MinimalSubscriber<modulo_core::EncodedState>>(this->sub_node_)->received_future, 500ms
    );
    EXPECT_EQ(recv_state->get_type(), expected_type);
    EXPECT_EQ(recv_state->get_parameter_type(), expected_parameter_type);
    EXPECT_EQ(recv_state->is_empty(), expected_empty);
    EXPECT_EQ(recv_state->get_name(), expected_name);
    if (test_function) {
      test_function(pub_state, recv_state);
    }
    this->clear_nodes();
  }

  template<typename PubT, typename RecvT>
  void communicate_basic_parameter(const PubT& first_value, const RecvT& second_value) {
    // test with empty Parameter
    this->communicate<Parameter<PubT>, Parameter<RecvT>>(Parameter<PubT>("this"), Parameter<RecvT>("that"), true);
    this->communicate<Parameter<std::vector<PubT>>, Parameter<std::vector<RecvT>>>(
        Parameter<std::vector<PubT>>("this"), Parameter<std::vector<RecvT>>("that"), true
    );
    // test with filled Parameter
    this->communicate<Parameter<PubT>, Parameter<RecvT>>(
        Parameter<PubT>("this", first_value), Parameter<RecvT>("that", second_value), true,
        [](const std::shared_ptr<Parameter<PubT>>& pub_param, const std::shared_ptr<Parameter<RecvT>>& recv_param) {
          EXPECT_EQ(pub_param->get_value(), recv_param->get_value());
        }
    );
    this->communicate<Parameter<std::vector<PubT>>, Parameter<std::vector<RecvT>>>(
        Parameter<std::vector<PubT>>("this", {first_value, first_value}),
        Parameter<std::vector<RecvT>>("that", {second_value}), true, [](
            const std::shared_ptr<Parameter<std::vector<PubT>>>& pub_param,
            const std::shared_ptr<Parameter<std::vector<RecvT>>>& recv_param
        ) {
          ASSERT_EQ(pub_param->get_value().size(), recv_param->get_value().size());
          for (std::size_t i = 0; i < recv_param->get_value().size(); ++i) {
            EXPECT_EQ(pub_param->get_value().at(i), recv_param->get_value().at(i));
          }
        }
    );
  }
};

TEST_F(EncodedParameterCommunicationTest, SameType) {
  this->communicate_basic_parameter<bool, bool>(false, true);
  this->communicate_basic_parameter<int, int>(1, 2);
  this->communicate_basic_parameter<double, double>(0.2, 1.1);
  this->communicate_basic_parameter<std::string, std::string>("modulo", "core");
  this->communicate<Parameter<Eigen::VectorXd>, Parameter<Eigen::VectorXd>>(
      Parameter<Eigen::VectorXd>("this", Eigen::VectorXd::Random(2)),
      Parameter<Eigen::VectorXd>("that", Eigen::VectorXd::Random(3)), true, [](
          const std::shared_ptr<Parameter<Eigen::VectorXd>>& pub_param,
          const std::shared_ptr<Parameter<Eigen::VectorXd>>& recv_param
      ) {
        EXPECT_EQ(pub_param->get_value().size(), recv_param->get_value().size());
        EXPECT_TRUE(pub_param->get_value().isApprox(recv_param->get_value()));
      }
  );
  this->communicate<Parameter<Eigen::MatrixXd>, Parameter<Eigen::MatrixXd>>(
      Parameter<Eigen::MatrixXd>("this", Eigen::MatrixXd::Random(2, 3)),
      Parameter<Eigen::MatrixXd>("that", Eigen::MatrixXd::Random(3, 3)), true, [](
          const std::shared_ptr<Parameter<Eigen::MatrixXd>>& pub_param,
          const std::shared_ptr<Parameter<Eigen::MatrixXd>>& recv_param
      ) {
        EXPECT_EQ(pub_param->get_value().size(), recv_param->get_value().size());
        EXPECT_TRUE(pub_param->get_value().isApprox(recv_param->get_value()));
      }
  );
}

TEST_F(EncodedParameterCommunicationTest, IncompatibleStateType) {
//  this->communicate<State, Parameter<bool>>(State(StateType::STATE, "this"), Parameter<bool>("that", false), false);
//  this->communicate<State, Parameter<std::vector<bool>>>(
//      State(StateType::STATE, "this"), Parameter<std::vector<bool>>("that", {false}), false
//  );
//  this->communicate<State, Parameter<int>>(State(StateType::STATE, "this"), Parameter<int>("that", 1), false);
//  this->communicate<State, Parameter<std::vector<int>>>(
//      State(StateType::STATE, "this"), Parameter<std::vector<int>>("that", {1}), false
//  );
//  this->communicate<State, Parameter<double>>(State(StateType::STATE, "this"), Parameter<double>("that", 0.1), false);
//  this->communicate<State, Parameter<std::vector<double>>>(
//      State(StateType::STATE, "this"), Parameter<std::vector<double>>("that", {0.1}), false
//  );
//  this->communicate<State, Parameter<std::string>>(
//      State(StateType::STATE, "this"), Parameter<std::string>("that", "modulo"), false
//  );
//  this->communicate<State, Parameter<std::vector<std::string>>>(
//      State(StateType::STATE, "this"), Parameter<std::vector<std::string>>("that", {"modulo"}), false
//  );

  this->communicate<ParameterInterface, Parameter<int>>(
      ParameterInterface("this", ParameterType::BOOL), Parameter<int>("that", 1), true, [](
          const std::shared_ptr<ParameterInterface>& pub_param,
          const std::shared_ptr<Parameter<int>>& recv_param
      ) {
//        EXPECT_EQ(pub_param->);
//        EXPECT_TRUE(pub_param->get_value().isApprox(recv_param->get_value()));
      });
}
