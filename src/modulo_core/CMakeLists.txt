cmake_minimum_required(VERSION 3.5)
project(modulo_core)

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
find_package(ament_cmake_auto REQUIRED)
find_package(Protobuf REQUIRED)

ament_auto_find_build_dependencies()

set(CORE_SOURCES
    src/Cell.cpp
    src/MotionGenerator.cpp
    src/Controller.cpp
    src/StateConversion.cpp
    src/Visualizer.cpp
    src/SensorInterface.cpp
    src/RobotInterface.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED ${CORE_SOURCES})
target_link_libraries(${PROJECT_NAME}
  state_representation
)

ament_auto_add_executable(${PROJECT_NAME}_utest_architecture_cartesian_space tests/testArchitectureCartesianSpace.cpp)
target_link_libraries(${PROJECT_NAME}_utest_architecture_cartesian_space ${PROJECT_NAME} protocol_buffers ${PROTOBUF_LIBRARIES})

ament_auto_add_executable(${PROJECT_NAME}_utest_architecture_joint_space tests/testArchitectureJointSpace.cpp)
target_link_libraries(${PROJECT_NAME}_utest_architecture_joint_space ${PROJECT_NAME} protocol_buffers ${PROTOBUF_LIBRARIES})

ament_auto_add_executable(${PROJECT_NAME}_utest_architecture_dual_quaternion_space tests/testArchitectureDualQuaternionSpace.cpp)
target_link_libraries(${PROJECT_NAME}_utest_architecture_dual_quaternion_space ${PROJECT_NAME} protocol_buffers ${PROTOBUF_LIBRARIES})

ament_auto_package()