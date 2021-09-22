#!/usr/bin/python
import sys

import launch
import launch.actions
import launch.substitutions
import launch.events

from launch_ros import get_default_launch_description
import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg

def generate_launch_description():
    """Main."""
    ld = launch.LaunchDescription()

    # Prepare the visualization node.
    visualizer_node = launch_ros.actions.LifecycleNode(
        node_name='visualizer',
        package='modulo_core', 
        node_executable="modulo_core_test_cartesian", 
        output='screen')

    # Prepare the robot node.
    robot_interface_node = launch_ros.actions.LifecycleNode(
        node_name='robot_interface',
        package='modulo_core', 
        node_executable="modulo_core_test_cartesian", 
        output='screen')

    # Prepare the motion generator node.
    motion_generator_node = launch_ros.actions.LifecycleNode(
        node_name='motion_generator',
        package='modulo_core',
        node_executable="modulo_core_test_cartesian", 
        output='screen')


    # Make the visualization reach the configure
    emit_event_to_request_visualization_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(visualizer_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the robot_interface reach the configure
    emit_event_to_request_robot_interface_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(robot_interface_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the motion_generator reach the configure
    emit_event_to_request_motion_generator_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(motion_generator_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the visualization reach the activate
    emit_event_to_request_visualization_activate = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(visualizer_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    # Make the robot_interface reach the configure
    emit_event_to_request_robot_interface_activate = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(robot_interface_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    # Make the motion_generator reach the configure
    emit_event_to_request_motion_generator_activate = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(motion_generator_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    # Add the actions to the launch description.
    # The order they are added reflects the order in which they will be executed.
    ld.add_action(visualizer_node)
    ld.add_action(robot_interface_node)
    ld.add_action(motion_generator_node)
    ld.add_action(emit_event_to_request_visualization_configure)
    ld.add_action(emit_event_to_request_robot_interface_configure)
    ld.add_action(emit_event_to_request_motion_generator_configure)
    ld.add_action(emit_event_to_request_visualization_activate)
    ld.add_action(emit_event_to_request_robot_interface_activate)
    ld.add_action(emit_event_to_request_motion_generator_activate)

    # print('Starting introspection of launch description...')
    # print('')

    # print(launch.LaunchIntrospector().format_launch_description(ld))

    # print('')
    # print('Starting launch of launch description...')
    # print('')

    # # ls = launch.LaunchService(argv=argv, debug=True)
    # ls = launch.LaunchService(argv=argv)
    # ls.include_launch_description(get_default_launch_description())
    # ls.include_launch_description(ld)
    return ld


# if __name__ == '__main__':
#     main()