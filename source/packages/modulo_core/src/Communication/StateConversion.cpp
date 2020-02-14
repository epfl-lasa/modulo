#include "modulo_core/Communication/StateConversion.hpp"
#include "state_representation/Exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/Exceptions/EmptyStateException.hpp"

using namespace StateRepresentation::Exceptions;

namespace Modulo
{
	namespace Core
	{
		namespace Communication
		{
			namespace StateConversion
			{
				// read_msg functions
				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Pose & msg)
				{
					// transform messages
					Eigen::Vector3d position(msg.position.x, msg.position.y, msg.position.z);
					Eigen::Quaterniond orientation(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
					// read_msg the state
					state.set_position(position);
					state.set_orientation(orientation);
				}

				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::PoseStamped & msg)
				{
					if(state.get_reference_frame() != msg.header.frame_id)
					{
						throw IncompatibleReferenceFramesException(state.get_name() + " expected in " + state.get_reference_frame() + ", got " + msg.header.frame_id);
					}
					read_msg(state, msg.pose);
				}

				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Transform & msg)
				{
					// transform messages
					Eigen::Vector3d position(msg.translation.x, msg.translation.y, msg.translation.z);
					Eigen::Quaterniond orientation(msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z);
					// read_msg the state
					state.set_position(position);
					state.set_orientation(orientation);
				}

				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::TransformStamped & msg)
				{
					if(state.get_reference_frame() != msg.header.frame_id)
					{
						throw IncompatibleReferenceFramesException(state.get_name() + " expected in " + state.get_reference_frame() + ", got " + msg.header.frame_id);
					}
					state.set_name(msg.child_frame_id);
					read_msg(state, msg.transform);
				}

				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Twist & msg)
				{
					// transform messages
					Eigen::Vector3d linear_velocity(msg.linear.x, msg.linear.y, msg.linear.z);
					Eigen::Vector3d angular_velocity(msg.angular.x, msg.angular.y, msg.angular.z);
					// read_msg the state
					state.set_linear_velocity(linear_velocity);
					state.set_angular_velocity(angular_velocity);
				}

				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::TwistStamped & msg)
				{
					if(state.get_reference_frame() != msg.header.frame_id)
					{
						throw IncompatibleReferenceFramesException(state.get_name() + " expected in " + state.get_reference_frame() + ", got " + msg.header.frame_id);
					}
					read_msg(state, msg.twist);
				}

				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Accel & msg)
				{
					// transform messages
					Eigen::Vector3d linear_acceleration(msg.linear.x, msg.linear.y, msg.linear.z);
					Eigen::Vector3d angular_acceleration(msg.angular.x, msg.angular.y, msg.angular.z);
					// read_msg the state
					state.set_linear_acceleration(linear_acceleration);
					state.set_angular_acceleration(angular_acceleration);
				}

				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::AccelStamped & msg)
				{
					if(state.get_reference_frame() != msg.header.frame_id)
					{
						throw IncompatibleReferenceFramesException(state.get_name() + " expected in " + state.get_reference_frame() + ", got " + msg.header.frame_id);
					}
					read_msg(state, msg.accel);
				}

				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::Wrench & msg)
				{
					// transform messages
					Eigen::Vector3d force(msg.force.x, msg.force.y, msg.force.z);
					Eigen::Vector3d torque(msg.torque.x, msg.torque.y, msg.torque.z);
					// read_msg the state
					state.set_force(force);
					state.set_torque(torque);
				}

				void read_msg(StateRepresentation::CartesianState & state, const geometry_msgs::msg::WrenchStamped & msg)
				{
					if(state.get_reference_frame() != msg.header.frame_id)
					{
						throw IncompatibleReferenceFramesException(state.get_name() + " expected in " + state.get_reference_frame() + ", got " + msg.header.frame_id);
					}
					read_msg(state, msg.wrench);
				}

				void read_msg(StateRepresentation::CartesianState & state, const nav_msgs::msg::Odometry & msg)
				{
					// transform messages
					// Pose
					Eigen::Vector3d position(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
					Eigen::Quaterniond orientation(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
					// Twist
					Eigen::Vector3d linear_velocity(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
					Eigen::Vector3d angular_velocity(msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z);

					// read_msg the state
					// Pose
					state.set_position(position);
					state.set_orientation(orientation);
					// Twist
					state.set_linear_velocity(linear_velocity);
					state.set_angular_velocity(angular_velocity);
				}

				void read_msg(StateRepresentation::JointState & state, const sensor_msgs::msg::JointState & msg)
				{
					state.set_names(msg.name);
					state.set_positions(Eigen::VectorXd::Map(msg.position.data(), msg.position.size()));
					state.set_velocities(Eigen::VectorXd::Map(msg.velocity.data(), msg.velocity.size()));
					state.set_torques(Eigen::VectorXd::Map(msg.effort.data(), msg.effort.size()));
				}

				void read_msg(StateRepresentation::JacobianMatrix & state, const modulo_msgs::msg::JacobianMatrix & msg)
				{
					state.set_nb_rows(msg.nb_dimensions);
					state.set_nb_cols(msg.nb_joints);
					state.set_joint_names(msg.joint_names);
					state.set_data(Eigen::MatrixXd::Map(msg.data.data(), msg.nb_dimensions, msg.nb_joints)); 
				}

				void read_msg(StateRepresentation::DualQuaternionPose & state, const geometry_msgs::msg::Pose & msg)
				{
					Eigen::Quaterniond orientation(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
					Eigen::Vector3d position(msg.position.x, msg.position.y, msg.position.z);
					state.set_orientation(orientation);
					state.set_position(position);
				}

				void read_msg(StateRepresentation::DualQuaternionPose & state, const geometry_msgs::msg::PoseStamped & msg)
				{
					if(state.get_reference_frame() != msg.header.frame_id)
					{
						throw IncompatibleReferenceFramesException(state.get_name() + " expected in " + state.get_reference_frame() + ", got " + msg.header.frame_id);
					}
					read_msg(state, msg.pose);
				}

				void read_msg(StateRepresentation::DualQuaternionTwist & state, const geometry_msgs::msg::Twist & msg)
				{
					state.set_linear_velocity(Eigen::Vector3d(msg.linear.x, msg.linear.y, msg.linear.z));
					state.set_angular_velocity(Eigen::Vector3d(msg.angular.x, msg.angular.y, msg.angular.z));
				}

				void read_msg(StateRepresentation::DualQuaternionTwist & state, const geometry_msgs::msg::TwistStamped & msg)
				{
					if(state.get_reference_frame() != msg.header.frame_id)
					{
						throw IncompatibleReferenceFramesException(state.get_name() + " expected in " + state.get_reference_frame() + ", got " + msg.header.frame_id);
					}
					read_msg(state, msg.twist);
				}

				void read_msg(StateRepresentation::Parameter<double> & state, const std_msgs::msg::Float64 & msg)
				{
					state.set_value(msg.data);
				}

				// write_msg functions
				void write_msg(geometry_msgs::msg::Quaternion & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time &)
				{
					if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
					// orientation
					msg.w = state.get_orientation().w();
					msg.x = state.get_orientation().x();
					msg.y = state.get_orientation().y();
					msg.z = state.get_orientation().z();
				}

				void write_msg(geometry_msgs::msg::Pose & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time &)
				{
					if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
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

				void write_msg(geometry_msgs::msg::PoseStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time)
				{
					msg.header.stamp = time;
					msg.header.frame_id = state.get_reference_frame();
					write_msg(msg.pose, state, time);
				}

				void write_msg(geometry_msgs::msg::Transform & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time &)
				{
					if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
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

				void write_msg(geometry_msgs::msg::TransformStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time)
				{
					msg.header.stamp = time;
					msg.header.frame_id = state.get_reference_frame();
					msg.child_frame_id = state.get_name();
					write_msg(msg.transform, state, time);
				}

				void write_msg(geometry_msgs::msg::Twist & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time &)
				{
					if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
					// linear velocity
					msg.linear.x = state.get_linear_velocity()(0);
					msg.linear.y = state.get_linear_velocity()(1);
					msg.linear.z = state.get_linear_velocity()(2);
					// angular velocity
					msg.angular.x = state.get_angular_velocity()(0);
					msg.angular.y = state.get_angular_velocity()(1);
					msg.angular.z = state.get_angular_velocity()(2);
				}

				void write_msg(geometry_msgs::msg::TwistStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time)
				{
					msg.header.stamp = time;
					msg.header.frame_id = state.get_reference_frame();
					write_msg(msg.twist, state, time);
				}

				void write_msg(geometry_msgs::msg::Accel & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time &)
				{
					if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
					// linear acceleration
					msg.linear.x = state.get_linear_acceleration()(0);
					msg.linear.y = state.get_linear_acceleration()(1);
					msg.linear.z = state.get_linear_acceleration()(2);
					// angular acceleration
					msg.angular.x = state.get_angular_acceleration()(0);
					msg.angular.y = state.get_angular_acceleration()(1);
					msg.angular.z = state.get_angular_acceleration()(2);
				}

				void write_msg(geometry_msgs::msg::AccelStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time)
				{
					msg.header.stamp = time;
					msg.header.frame_id = state.get_reference_frame();
					write_msg(msg.accel, state, time);
				}

				void write_msg(geometry_msgs::msg::Wrench & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time &)
				{
					if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
					// force
					msg.force.x = state.get_force()(0);
					msg.force.y = state.get_force()(1);
					msg.force.z = state.get_force()(2);
					// torque
					msg.torque.x = state.get_torque()(0);
					msg.torque.y = state.get_torque()(1);
					msg.torque.z = state.get_torque()(2);
				}

				void write_msg(geometry_msgs::msg::WrenchStamped & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time)
				{
					msg.header.stamp = time;
					msg.header.frame_id = state.get_reference_frame();
					write_msg(msg.wrench, state, time);
				}

				void write_msg(sensor_msgs::msg::JointState & msg, const StateRepresentation::JointState & state, const rclcpp::Time & time)
				{
					if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
					msg.header.stamp = time;
					msg.name = state.get_names();
					msg.position = std::vector<double>(state.get_positions().data(), state.get_positions().data() + state.get_positions().size());
					msg.velocity = std::vector<double>(state.get_velocities().data(), state.get_velocities().data() + state.get_velocities().size());
					msg.effort = std::vector<double>(state.get_torques().data(), state.get_torques().data() + state.get_torques().size());
				}

				void write_msg(modulo_msgs::msg::JacobianMatrix & msg, const StateRepresentation::JacobianMatrix & state, const rclcpp::Time & time)
				{
					if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
					msg.header.stamp = time;
					msg.nb_dimensions = state.get_nb_rows();
					msg.nb_joints = state.get_nb_cols();
					msg.joint_names = state.get_joint_names(); 
					msg.data = std::vector<double>(state.get_data().data(), state.get_data().data() + state.get_data().size());
				}

				void write_msg(geometry_msgs::msg::Pose & msg, const StateRepresentation::DualQuaternionPose & state, const rclcpp::Time &)
				{
					if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
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

				void write_msg(geometry_msgs::msg::PoseStamped & msg, const StateRepresentation::DualQuaternionPose & state, const rclcpp::Time & time)
				{
					msg.header.stamp = time;
					msg.header.frame_id = state.get_reference_frame();
					write_msg(msg.pose, state, time);
				}

				void write_msg(geometry_msgs::msg::Twist & msg, const StateRepresentation::DualQuaternionTwist & state, const rclcpp::Time &)
				{
					if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
					Eigen::Vector3d linear = state.get_linear_velocity();
					Eigen::Vector3d angular = state.get_angular_velocity();

					msg.linear.x = linear(0);
					msg.linear.y = linear(1);
					msg.linear.z = linear(2);

					msg.angular.x = angular(0);
					msg.angular.y = angular(1);
					msg.angular.z = angular(2);
				}

				void write_msg(geometry_msgs::msg::TwistStamped & msg, const StateRepresentation::DualQuaternionTwist & state, const rclcpp::Time & time)
				{
					msg.header.stamp = time;
					msg.header.frame_id = state.get_reference_frame();
					write_msg(msg.twist, state, time);
				}

				void write_msg(tf2_msgs::msg::TFMessage & msg, const StateRepresentation::CartesianState & state, const rclcpp::Time & time)
				{
					if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
					geometry_msgs::msg::TransformStamped transform;
					write_msg(transform, state, time);
					msg.transforms.push_back(transform);
				}

				void write_msg(std_msgs::msg::Float64MultiArray & msg, const StateRepresentation::CartesianTwist & state, const rclcpp::Time &)
				{
					if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
					Eigen::Matrix<double, 6, 1> twist;
					twist << state.get_angular_velocity(), state.get_linear_velocity();
					msg.data = std::vector<double>(twist.data(), twist.data() + twist.size());		
				}

				void write_msg(std_msgs::msg::Float64 & msg, const StateRepresentation::Parameter<double> & state, const rclcpp::Time &)
				{
					if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty while attempting to publish it");
					msg.data = state.get_value();
				}
			}
		}
	}
}