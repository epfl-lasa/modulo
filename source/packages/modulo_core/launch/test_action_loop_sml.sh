#!/bin/bash
ros2 lifecycle set /robot_interface configure
ros2 lifecycle set /move_home configure
ros2 lifecycle set /move_action1 configure
ros2 lifecycle set /move_action2 configure
ros2 lifecycle set /monitor configure
ros2 lifecycle set /event_handler configure

ros2 lifecycle set /robot_interface activate
#ros2 lifecycle set /move_home activate
ros2 lifecycle set /monitor activate
ros2 lifecycle set /event_handler activate