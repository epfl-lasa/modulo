#include "modulo_core/StateConversion.hpp"
#include "state_representation/Exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/Exceptions/EmptyStateException.hpp"

using namespace StateRepresentation::Exceptions;

namespace Modulo
{
	namespace Core
	{
		namespace StateConversion
		{
			// update functions
			void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Pose & msg)
			{
				// transform messages
				Eigen::Vector3d position(msg.position.x, msg.position.y, msg.position.z);
				Eigen::Quaterniond orientation(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
				// update the state
				state.set_position(position);
				state.set_orientation(orientation);
			}

			void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::PoseStamped & msg)
			{
				if(state.get_reference_frame() != msg.header.frame_id)
				{
					throw IncompatibleReferenceFramesException(state.get_name() + " expected in " + state.get_reference_frame() + ", got " + msg.header.frame_id);
				}
				update(state, msg.pose);
			}

			void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Transform & msg)
			{
				// transform messages
				Eigen::Vector3d position(msg.translation.x, msg.translation.y, msg.translation.z);
				Eigen::Quaterniond orientation(msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z);
				// update the state
				state.set_position(position);
				state.set_orientation(orientation);
			}

			void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::TransformStamped & msg)
			{
				if(state.get_reference_frame() != msg.header.frame_id)
				{
					throw IncompatibleReferenceFramesException(state.get_name() + " expected in " + state.get_reference_frame() + ", got " + msg.header.frame_id);
				}
				state.set_name(msg.child_frame_id);
				update(state, msg.transform);
			}

			void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Twist & msg)
			{
				// transform messages
				Eigen::Vector3d linear_velocity(msg.linear.x, msg.linear.y, msg.linear.z);
				Eigen::Vector3d angular_velocity(msg.angular.x, msg.angular.y, msg.angular.z);
				// update the state
				state.set_linear_velocity(linear_velocity);
				state.set_angular_velocity(angular_velocity);
			}

			void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::TwistStamped & msg)
			{
				if(state.get_reference_frame() != msg.header.frame_id)
				{
					throw IncompatibleReferenceFramesException(state.get_name() + " expected in " + state.get_reference_frame() + ", got " + msg.header.frame_id);
				}
				update(state, msg.twist);
			}

			void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Accel & msg)
			{
				// transform messages
				Eigen::Vector3d linear_acceleration(msg.linear.x, msg.linear.y, msg.linear.z);
				Eigen::Vector3d angular_acceleration(msg.angular.x, msg.angular.y, msg.angular.z);
				// update the state
				state.set_linear_acceleration(linear_acceleration);
				state.set_angular_acceleration(angular_acceleration);
			}

			void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::AccelStamped & msg)
			{
				if(state.get_reference_frame() != msg.header.frame_id)
				{
					throw IncompatibleReferenceFramesException(state.get_name() + " expected in " + state.get_reference_frame() + ", got " + msg.header.frame_id);
				}
				update(state, msg.accel);
			}

			void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Wrench & msg)
			{
				// transform messages
				Eigen::Vector3d force(msg.force.x, msg.force.y, msg.force.z);
				Eigen::Vector3d torque(msg.torque.x, msg.torque.y, msg.torque.z);
				// update the state
				state.set_force(force);
				state.set_torque(torque);
			}

			void update(StateRepresentation::CartesianState & state, const geometry_msgs::msg::WrenchStamped & msg)
			{
				if(state.get_reference_frame() != msg.header.frame_id)
				{
					throw IncompatibleReferenceFramesException(state.get_name() + " expected in " + state.get_reference_frame() + ", got " + msg.header.frame_id);
				}
				update(state, msg.wrench);
			}

			void update(StateRepresentation::JointState & state, const sensor_msgs::msg::JointState & msg)
			{
				state.set_names(msg.name);
				state.set_positions(Eigen::VectorXd::Map(msg.position.data(), msg.position.size()));
				state.set_velocities(Eigen::VectorXd::Map(msg.velocity.data(), msg.velocity.size()));
				state.set_torques(Eigen::VectorXd::Map(msg.effort.data(), msg.effort.size()));
			}

			void update(StateRepresentation::JacobianMatrix & state, const modulo_msgs::msg::JacobianMatrix & msg)
			{
				state.set_nb_rows(msg.nb_dimensions);
				state.set_nb_cols(msg.nb_joints);
				state.set_data(Eigen::MatrixXd::Map(msg.data.data(), msg.nb_dimensions, msg.nb_joints)); 
			}

			void update(StateRepresentation::DualQuaternionPose & state, const geometry_msgs::msg::Pose & msg)
			{
				Eigen::Quaterniond orientation(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
				Eigen::Vector3d position(msg.position.x, msg.position.y, msg.position.z);
				state.set_orientation(orientation);
				state.set_position(position);
			}

			void update(StateRepresentation::DualQuaternionPose & state, const geometry_msgs::msg::PoseStamped & msg)
			{
				if(state.get_reference_frame() != msg.header.frame_id)
				{
					throw IncompatibleReferenceFramesException(state.get_name() + " expected in " + state.get_reference_frame() + ", got " + msg.header.frame_id);
				}
				update(state, msg.pose);
			}

			void update(StateRepresentation::DualQuaternionTwist & state, const geometry_msgs::msg::Twist & msg)
			{
				state.set_linear_velocity(Eigen::Vector3d(msg.linear.x, msg.linear.y, msg.linear.z));
				state.set_angular_velocity(Eigen::Vector3d(msg.angular.x, msg.angular.y, msg.angular.z));
			}

			void update(StateRepresentation::DualQuaternionTwist & state, const geometry_msgs::msg::TwistStamped & msg)
			{
				if(state.get_reference_frame() != msg.header.frame_id)
				{
					throw IncompatibleReferenceFramesException(state.get_name() + " expected in " + state.get_reference_frame() + ", got " + msg.header.frame_id);
				}
				update(state, msg.twist);
			}

			// extract functions
			void extract(geometry_msgs::msg::Quaternion & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time &)
			{
				if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
				// orientation
				msg.w = state.get_orientation().w();
				msg.x = state.get_orientation().x();
				msg.y = state.get_orientation().y();
				msg.z = state.get_orientation().z();
			}

			void extract(geometry_msgs::msg::Pose & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time &)
			{
				if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
				// position
				msg.position.x = state.get_position()(0);
				msg.position.y = state.get_position()(1);
				msg.position.z = state.get_position()(2);
				// orientation
				msg.orientation.w = state.get_orientation().w();
				msg.orientation.x = state.get_orientation().x();
				msg.orientation.y = state.get_orientation().y();
				msg.orientation.z = state.get_orientation().z();
			}

			void extract(geometry_msgs::msg::PoseStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time)
			{
				msg.header.stamp = time;
				msg.header.frame_id = state.get_reference_frame();
				extract(msg.pose, state, time);
			}

			void extract(geometry_msgs::msg::Transform & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time &)
			{
				if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
				// position 
				msg.translation.x = state.get_position()(0);
				msg.translation.y = state.get_position()(1);
				msg.translation.z = state.get_position()(2);
				// orientation
				msg.rotation.w = state.get_orientation().w();
				msg.rotation.x = state.get_orientation().x();
				msg.rotation.y = state.get_orientation().y();
				msg.rotation.z = state.get_orientation().z();
			}

			void extract(geometry_msgs::msg::TransformStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time)
			{
				msg.header.stamp = time;
				msg.header.frame_id = state.get_reference_frame();
				msg.child_frame_id = state.get_name();
				extract(msg.transform, state, time);
			}

			void extract(geometry_msgs::msg::Twist & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time &)
			{
				if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
				// linear velocity
				msg.linear.x = state.get_linear_velocity()(0);
				msg.linear.y = state.get_linear_velocity()(1);
				msg.linear.z = state.get_linear_velocity()(2);
				// angular velocity
				msg.angular.x = state.get_angular_velocity()(0);
				msg.angular.y = state.get_angular_velocity()(1);
				msg.angular.z = state.get_angular_velocity()(2);
			}

			void extract(geometry_msgs::msg::TwistStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time)
			{
				msg.header.stamp = time;
				msg.header.frame_id = state.get_reference_frame();
				extract(msg.twist, state, time);
			}

			void extract(geometry_msgs::msg::Accel & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time &)
			{
				if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
				// linear acceleration
				msg.linear.x = state.get_linear_acceleration()(0);
				msg.linear.y = state.get_linear_acceleration()(1);
				msg.linear.z = state.get_linear_acceleration()(2);
				// angular acceleration
				msg.angular.x = state.get_angular_acceleration()(0);
				msg.angular.y = state.get_angular_acceleration()(1);
				msg.angular.z = state.get_angular_acceleration()(2);
			}

			void extract(geometry_msgs::msg::AccelStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time)
			{
				msg.header.stamp = time;
				msg.header.frame_id = state.get_reference_frame();
				extract(msg.accel, state, time);
			}

			void extract(geometry_msgs::msg::Wrench & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time &)
			{
				if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
				// force
				msg.force.x = state.get_force()(0);
				msg.force.y = state.get_force()(1);
				msg.force.z = state.get_force()(2);
				// torque
				msg.torque.x = state.get_torque()(0);
				msg.torque.y = state.get_torque()(1);
				msg.torque.z = state.get_torque()(2);
			}

			void extract(geometry_msgs::msg::WrenchStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time)
			{
				msg.header.stamp = time;
				msg.header.frame_id = state.get_reference_frame();
				extract(msg.wrench, state, time);
			}

			void extract(sensor_msgs::msg::JointState & msg, const StateRepresentation::JointState & state, const rclcpp::Time & time)
			{
				if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
				msg.header.stamp = time;
				msg.name = state.get_names();
				msg.position = std::vector<double>(state.get_positions().data(), state.get_positions().data() + state.get_positions().size());
				msg.velocity = std::vector<double>(state.get_velocities().data(), state.get_velocities().data() + state.get_velocities().size());
				msg.effort = std::vector<double>(state.get_torques().data(), state.get_torques().data() + state.get_torques().size());
			}

			void extract(modulo_msgs::msg::JacobianMatrix & msg, const StateRepresentation::JacobianMatrix & state, const rclcpp::Time & time)
			{
				if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
				msg.header.stamp = time;
				msg.nb_dimensions = state.get_nb_rows();
				msg.nb_joints = state.get_nb_cols();
				msg.data = std::vector<double>(state.get_data().data(), state.get_data().data() + state.get_data().size());
			}

			void extract(geometry_msgs::msg::Pose & msg, const StateRepresentation::DualQuaternionPose & state, const rclcpp::Time &)
			{
				if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
				// position
				msg.position.x = state.get_position()(0);
				msg.position.y = state.get_position()(1);
				msg.position.z = state.get_position()(2);
				// orientation
				msg.orientation.w = state.get_orientation().w();
				msg.orientation.x = state.get_orientation().x();
				msg.orientation.y = state.get_orientation().y();
				msg.orientation.z = state.get_orientation().z();
			}

			void extract(geometry_msgs::msg::PoseStamped & msg, const StateRepresentation::DualQuaternionPose & state, const rclcpp::Time & time)
			{
				msg.header.stamp = time;
				msg.header.frame_id = state.get_reference_frame();
				extract(msg.pose, state, time);
			}

			void extract(geometry_msgs::msg::Twist & msg, const StateRepresentation::DualQuaternionTwist & state, const rclcpp::Time &)
			{
				if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
				Eigen::Vector3d linear = state.get_linear_velocity();
				Eigen::Vector3d angular = state.get_angular_velocity();

				msg.linear.x = linear(0);
				msg.linear.y = linear(1);
				msg.linear.z = linear(2);

				msg.angular.x = angular(0);
				msg.angular.y = angular(1);
				msg.angular.z = angular(2);
			}

			void extract(geometry_msgs::msg::TwistStamped & msg, const StateRepresentation::DualQuaternionTwist & state, const rclcpp::Time & time)
			{
				msg.header.stamp = time;
				msg.header.frame_id = state.get_reference_frame();
				extract(msg.twist, state, time);
			}

			void extract(tf2_msgs::msg::TFMessage & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time)
			{
				if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
				geometry_msgs::msg::TransformStamped transform;
				extract(transform, state, time);
				msg.transforms.push_back(transform);
			}
		}
	}
}