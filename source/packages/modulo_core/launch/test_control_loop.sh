#!/bin/bash
# ros2 param set /robot_interface damping_eigenvalues "[10.0, 5.0, 5.0, 2.0, 1.0, 1.0]"
ros2 param set /robot_interface damping_eigenvalues "[50.0, 25.0, 25.0, 2.0, 1.0, 1.0]"

ros2 lifecycle set /robot_interface configure
ros2 lifecycle set /motion_generator configure

ros2 lifecycle set /robot_interface activate
ros2 lifecycle set /motion_generator activate