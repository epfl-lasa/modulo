cmake_minimum_required(VERSION 3.15)
project(modulo_components)

# default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_auto_find_build_dependencies()

find_package(control_libraries 5.0.0 REQUIRED COMPONENTS state_representation)

include_directories(include)

ament_auto_add_library(${PROJECT_NAME} SHARED
    ${PROJECT_SOURCE_DIR}/src/ComponentInterface.cpp
    ${PROJECT_SOURCE_DIR}/src/Component.cpp
    ${PROJECT_SOURCE_DIR}/src/LifecycleComponent.cpp)

ament_auto_package()
