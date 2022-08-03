#include <gtest/gtest.h>

#include "test_modulo_core/communication_nodes.hpp"

#define tol 0.001

using namespace modulo_core::communication;
using namespace state_representation;

template<typename PubT, typename RecvT>
void test_cartesian_dist(
    const std::shared_ptr<PubT>& pub_state, const std::shared_ptr<RecvT>& recv_state,
    CartesianStateVariable distance_variable
) {
  EXPECT_EQ(pub_state->get_reference_frame(), recv_state->get_reference_frame());
  EXPECT_LT(pub_state->dist(*recv_state, distance_variable), tol);
}

template<typename PubT, typename RecvT>
void test_joint_dist(
    const std::shared_ptr<PubT>& pub_state, const std::shared_ptr<RecvT>& recv_state,
    JointStateVariable distance_variable
) {
  EXPECT_EQ(pub_state->get_size(), recv_state->get_size());
  EXPECT_LT(pub_state->dist(*recv_state, distance_variable), tol);
}

class EncodedCommunicationTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);

    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    clock_ = std::make_shared<rclcpp::Clock>();
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  void add_nodes(
      const std::string& topic_name, const std::shared_ptr<MessagePairInterface>& pub_message,
      const std::shared_ptr<MessagePairInterface>& sub_message
  ) {
    pub_node_ = std::make_shared<MinimalPublisher<modulo_core::EncodedState>>(topic_name, pub_message);
    sub_node_ = std::make_shared<MinimalSubscriber<modulo_core::EncodedState>>(topic_name, sub_message);
    exec_->add_node(pub_node_);
    exec_->add_node(sub_node_);
  }

  void clear_nodes() {
    pub_node_.reset();
    sub_node_.reset();
  }

  template<typename PubT, typename RecvT>
  void communicate(
      PubT publish_state, RecvT receive_state,
      std::function<void(const std::shared_ptr<PubT>&, const std::shared_ptr<RecvT>&)> test_function = {}
  ) {
    auto expected_type = receive_state.get_type();
    auto pub_state = std::make_shared<PubT>(publish_state);
    auto recv_state = std::make_shared<RecvT>(receive_state);
    auto pub_message = make_shared_message_pair(pub_state, this->clock_);
    auto recv_message = make_shared_message_pair(recv_state, this->clock_);
    this->add_nodes("/test_topic", pub_message, recv_message);
    this->exec_->template spin_until_future_complete(
        std::dynamic_pointer_cast<MinimalSubscriber<modulo_core::EncodedState>>(this->sub_node_)->received_future, 500ms
    );
    EXPECT_EQ(recv_state->get_type(), expected_type);
    EXPECT_EQ(recv_state->get_name(), pub_state->get_name());
    if (test_function) {
      test_function(pub_state, recv_state);
    }
    this->clear_nodes();
  }

  template<typename T>
  void communicate_same_cartesian_state(CartesianStateVariable distance_variable) {
    this->communicate<T, T>(
        T::Random("this", "world"), T::Identity("that", "base"),
        [distance_variable](const std::shared_ptr<T>& pub_state, const std::shared_ptr<T>& recv_state) {
          test_cartesian_dist(pub_state, recv_state, distance_variable);
        }
    );
  }

  template<typename T>
  void communicate_same_joint_state(JointStateVariable distance_variable) {
    this->communicate<T, T>(
        T::Random("this", 2), T::Zero("that", 3),
        [distance_variable](const std::shared_ptr<T>& pub_state, const std::shared_ptr<T>& recv_state) {
          test_joint_dist(pub_state, recv_state, distance_variable);
        }
    );
  }

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::shared_ptr<rclcpp::Node> pub_node_;
  std::shared_ptr<rclcpp::Node> sub_node_;
  std::shared_ptr<rclcpp::Clock> clock_;
};

TEST_F(EncodedCommunicationTest, SameType) {
  this->communicate<State, State>(
      State(StateType::STATE, "this"), State(StateType::STATE, "that"));
  this->communicate<SpatialState, SpatialState>(
      SpatialState(StateType::SPATIAL_STATE, "this", "world"), SpatialState(StateType::SPATIAL_STATE, "that", "base"),
      [](const std::shared_ptr<SpatialState>& pub_state, const std::shared_ptr<SpatialState>& recv_state) {
        EXPECT_EQ(pub_state->get_reference_frame(), recv_state->get_reference_frame());
      }
  );
  this->communicate_same_cartesian_state<CartesianState>(CartesianStateVariable::ALL);
  this->communicate_same_cartesian_state<CartesianPose>(CartesianStateVariable::POSE);
  this->communicate_same_cartesian_state<CartesianTwist>(CartesianStateVariable::TWIST);
  this->communicate_same_cartesian_state<CartesianAcceleration>(CartesianStateVariable::ACCELERATION);
  this->communicate_same_cartesian_state<CartesianWrench>(CartesianStateVariable::WRENCH);
  this->communicate_same_joint_state<JointState>(JointStateVariable::ALL);
  this->communicate_same_joint_state<JointPositions>(JointStateVariable::POSITIONS);
  this->communicate_same_joint_state<JointVelocities>(JointStateVariable::VELOCITIES);
  this->communicate_same_joint_state<JointAccelerations>(JointStateVariable::ACCELERATIONS);
  this->communicate_same_joint_state<JointTorques>(JointStateVariable::TORQUES);
}

TEST_F(EncodedCommunicationTest, EncodedStateSameType2) {
  using namespace state_representation;
  auto pub_state = std::make_shared<Parameter<double>>("this", 0.3);
  auto pub_message = make_shared_message_pair(pub_state, this->clock_);
  auto sub_state = std::make_shared<Parameter<double>>("that", 0.5);
  auto sub_message = make_shared_message_pair(sub_state, this->clock_);
  this->add_nodes("/test_topic", pub_message, sub_message);
  this->exec_->template spin_until_future_complete(
      std::dynamic_pointer_cast<MinimalSubscriber<modulo_core::EncodedState>>(this->sub_node_)->received_future, 500ms
  );

  EXPECT_EQ(pub_state->get_type(), sub_state->get_type());
  EXPECT_EQ(pub_state->get_name(), sub_state->get_name());
  EXPECT_EQ(pub_state->get_value(), sub_state->get_value());
}

TEST_F(EncodedCommunicationTest, EncodedStateIncompatibleType) {
  using namespace state_representation;
  auto pub_state = std::make_shared<CartesianPose>(CartesianPose::Random("this", "world"));
  auto pub_message = make_shared_message_pair(pub_state, this->clock_);
  auto sub_state = std::make_shared<JointState>(JointState::Zero("that", 3));
  auto sub_message = make_shared_message_pair(sub_state, this->clock_);
  this->add_nodes("/test_topic", pub_message, sub_message);
  this->exec_->template spin_until_future_complete(
      std::dynamic_pointer_cast<MinimalSubscriber<modulo_core::EncodedState>>(this->sub_node_)->received_future, 500ms
  );

  // An exception is thrown and caught, so the sub state didn't change
  EXPECT_EQ(sub_state->get_type(), StateType::JOINT_STATE);
  EXPECT_TRUE(sub_state->data().isApprox(JointState::Zero("that", 3).data()));
}

TEST_F(EncodedCommunicationTest, EncodedStateIncompatibleType2) {
  using namespace state_representation;
  auto pub_state = std::make_shared<Parameter<bool>>("this", true);
  auto pub_message = make_shared_message_pair(pub_state, this->clock_);
  auto sub_state = std::make_shared<Parameter<std::vector<std::string>>>("that", std::vector<std::string>{"1", "2"});
  auto sub_message = make_shared_message_pair(sub_state, this->clock_);
  this->add_nodes("/test_topic", pub_message, sub_message);
  this->exec_->template spin_until_future_complete(
      std::dynamic_pointer_cast<MinimalSubscriber<modulo_core::EncodedState>>(this->sub_node_)->received_future, 500ms
  );

  // An exception is thrown and caught, so the sub state didn't change
  std::cout << *sub_state << std::endl;
  EXPECT_EQ(sub_state->get_type(), StateType::PARAMETER);
  EXPECT_EQ(sub_state->get_parameter_type(), ParameterType::STRING_ARRAY);
//  EXPECT_TRUE(sub_state->data().isApprox(JointState::Zero("that", 3).data()));
}

TEST_F(EncodedCommunicationTest, EncodedStateCompatibleType) {
  using namespace state_representation;
  auto pub_state = std::make_shared<JointState>(JointState::Random("this", 3));
  auto pub_message = make_shared_message_pair(pub_state, this->clock_);
  auto sub_state = std::make_shared<JointPositions>(JointPositions::Zero("that", 3));
  auto sub_message = make_shared_message_pair(sub_state, this->clock_);
  this->add_nodes("/test_topic", pub_message, sub_message);
  this->exec_->template spin_until_future_complete(
      std::dynamic_pointer_cast<MinimalSubscriber<modulo_core::EncodedState>>(this->sub_node_)->received_future, 500ms
  );

  EXPECT_EQ(sub_state->get_type(), StateType::JOINT_POSITIONS);
  EXPECT_EQ(sub_state->get_name(), pub_state->get_name());
  EXPECT_TRUE(sub_state->get_positions().isApprox(pub_state->get_positions()));
}

TEST_F(EncodedCommunicationTest, EncodedStateCompatibleType2) {
  using namespace state_representation;
  auto pub_state = std::make_shared<JointPositions>(JointPositions::Random("this", 3));
  auto pub_message = make_shared_message_pair(pub_state, this->clock_);
  auto sub_state = std::make_shared<JointState>(JointState::Zero("that", 3));
  auto sub_message = make_shared_message_pair(sub_state, this->clock_);
  this->add_nodes("/test_topic", pub_message, sub_message);
  this->exec_->template spin_until_future_complete(
      std::dynamic_pointer_cast<MinimalSubscriber<modulo_core::EncodedState>>(this->sub_node_)->received_future, 500ms
  );

  EXPECT_EQ(sub_state->get_type(), StateType::JOINT_STATE);
  EXPECT_EQ(sub_state->get_name(), pub_state->get_name());
  EXPECT_TRUE(sub_state->get_positions().isApprox(pub_state->get_positions()));
}

TEST_F(EncodedCommunicationTest, EncodedStateCompatibleType3) {
  using namespace state_representation;
  auto pub_state = std::make_shared<CartesianState>(CartesianState::Random("this"));
  auto pub_message = make_shared_message_pair(pub_state, this->clock_);
  auto sub_state = std::make_shared<CartesianPose>(CartesianPose::Identity("that"));
  auto sub_message = make_shared_message_pair(sub_state, this->clock_);
  this->add_nodes("/test_topic", pub_message, sub_message);
  this->exec_->template spin_until_future_complete(
      std::dynamic_pointer_cast<MinimalSubscriber<modulo_core::EncodedState>>(this->sub_node_)->received_future, 500ms
  );

  EXPECT_EQ(sub_state->get_type(), StateType::CARTESIAN_POSE);
  EXPECT_EQ(sub_state->get_name(), pub_state->get_name());
  EXPECT_TRUE(sub_state->get_pose().isApprox(pub_state->get_pose()));
}

TEST_F(EncodedCommunicationTest, EncodedStateCompatibleType4) {
  using namespace state_representation;
  auto pub_state = std::make_shared<CartesianPose>(CartesianPose::Random("this"));
  auto pub_message = make_shared_message_pair(pub_state, this->clock_);
  auto sub_state = std::make_shared<CartesianState>(CartesianState::Identity("that"));
  auto sub_message = make_shared_message_pair(sub_state, this->clock_);
  EXPECT_NE(sub_state->get_type(), pub_state->get_type());
  this->add_nodes("/test_topic", pub_message, sub_message);
  this->exec_->template spin_until_future_complete(
      std::dynamic_pointer_cast<MinimalSubscriber<modulo_core::EncodedState>>(this->sub_node_)->received_future, 500ms
  );

  EXPECT_EQ(sub_state->get_type(), StateType::CARTESIAN_STATE);
  EXPECT_EQ(sub_state->get_name(), pub_state->get_name());
  EXPECT_TRUE(sub_state->get_pose().isApprox(pub_state->get_pose()));
}
