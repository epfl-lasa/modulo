#!/bin/bash
ros2 lifecycle set /simulated_object configure
ros2 lifecycle set /robot_interface configure
ros2 lifecycle set /visualizer configure
ros2 lifecycle set /motion_generator configure

ros2 lifecycle set /simulated_object activate
ros2 lifecycle set /robot_interface activate
ros2 lifecycle set /visualizer activate
ros2 lifecycle set /motion_generator activate