#!/bin/bash
ros2 lifecycle set /robot_interface configure
ros2 lifecycle set /visualizer configure
ros2 lifecycle set /move_action configure
ros2 lifecycle set /random_attractor configure
ros2 lifecycle set /monitor configure

ros2 lifecycle set /robot_interface activate
ros2 lifecycle set /visualizer activate
ros2 lifecycle set /move_action activate
ros2 lifecycle set /random_attractor activate
ros2 lifecycle set /monitor activate