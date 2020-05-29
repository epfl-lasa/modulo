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

ament_auto_find_build_dependencies()

set(CORE_SOURCES
    src/Communication/MessagePassing/ReadStateConversion.cpp
    src/Communication/MessagePassing/WriteStateConversion.cpp
    src/Communication/MessagePassing/TransformListenerHandler.cpp
    src/Cell.cpp
    src/Monitor.cpp
    src/Recorder.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED ${CORE_SOURCES})
target_link_libraries(${PROJECT_NAME}
  state_representation
  dynamical_systems
)

ament_auto_add_executable(${PROJECT_NAME}_test_cartesian tests/testCartesian.cpp)
target_link_libraries(${PROJECT_NAME}_test_cartesian ${PROJECT_NAME})

ament_auto_add_executable(${PROJECT_NAME}_test_cartesian_circular tests/testCartesianCircular.cpp)
target_link_libraries(${PROJECT_NAME}_test_cartesian_circular ${PROJECT_NAME})

ament_auto_add_executable(${PROJECT_NAME}_test_joints tests/testJoints.cpp)
target_link_libraries(${PROJECT_NAME}_test_joints ${PROJECT_NAME})

ament_auto_package()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)