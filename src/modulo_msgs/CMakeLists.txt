cmake_minimum_required(VERSION 3.5)
project(modulo_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(MSG_CORE "msg/JacobianMatrix.msg")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_CORE}
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

#rosidl_target_interfaces(${PROJECT_NAME}_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()
