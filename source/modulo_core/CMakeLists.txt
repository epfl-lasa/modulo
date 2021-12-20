cmake_minimum_required(VERSION 3.9)
project(modulo_core)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

list(APPEND CMAKE_PREFIX_PATH /opt/openrobots)
find_package(pinocchio REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(osqp REQUIRED)

find_library(protobuf REQUIRED)
find_library(clproto REQUIRED)
find_library(state_representation REQUIRED)
find_library(dynamical_systems REQUIRED)
find_library(controllers REQUIRED)
find_library(robot_model REQUIRED)

set(CORE_SOURCES
    src/communication/message_passing/ReadStateConversion.cpp
    src/communication/message_passing/WriteStateConversion.cpp
    src/communication/message_passing/TransformListenerHandler.cpp
    src/communication/message_passing/TransformBroadcasterHandler.cpp
    src/communication/message_passing/MessagePassingHandler.cpp
    src/communication/CommunicationHandler.cpp
    src/Cell.cpp
    src/Component.cpp
    src/Monitor.cpp
    src/Recorder.cpp
)

include_directories(
  include
  ${STATE_REPRESENTATION_INCLUDE_DIR}
  ${DYNAMICAL_SYSTEMS_INCLUDE_DIR}
  ${CONTROLLERS_INCLUDE_DIR}
  ${ROBOT_MODEL_INCLUDE_DIR}
  ${PINOCCHIO_INCLUDE_DIR}
  ${OsqpEigen_INCLUDE_DIR}
  /opt/openrobots/include
  ${CLPROTO_INCLUDE_DIR}
)

ament_auto_add_library(${PROJECT_NAME} SHARED ${CORE_SOURCES})
target_link_libraries(${PROJECT_NAME}
  ${PINOCCHIO_LIBRARIES}
  OsqpEigen::OsqpEigen
  osqp::osqp
  clproto
  protobuf
  state_representation
  dynamical_systems
  controllers
  robot_model
)

ament_auto_add_executable(examples_test_cartesian examples/testCartesian.cpp)
target_link_libraries(examples_test_cartesian ${PROJECT_NAME})

ament_auto_add_executable(examples_test_cartesian_circular examples/testCartesianCircular.cpp)
target_link_libraries(examples_test_cartesian_circular ${PROJECT_NAME})

ament_auto_add_executable(examples_test_helicoidal_trajectory examples/testHelicoidalTrajectory.cpp)
target_link_libraries(examples_test_helicoidal_trajectory ${PROJECT_NAME})

ament_auto_add_executable(examples_test_joints examples/testJoints.cpp)
target_link_libraries(examples_test_joints ${PROJECT_NAME})

ament_auto_add_executable(examples_test_moving_reference_frame examples/testMovingReferenceFrame.cpp)
target_link_libraries(examples_test_moving_reference_frame ${PROJECT_NAME})

ament_auto_package()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)