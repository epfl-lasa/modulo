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

find_package(control_libraries 5.1.0 REQUIRED COMPONENTS state_representation)
find_library(clproto REQUIRED)

include_directories(include)

ament_auto_add_library(${PROJECT_NAME} SHARED
    ${PROJECT_SOURCE_DIR}/src/ComponentInterface.cpp
    ${PROJECT_SOURCE_DIR}/src/Component.cpp
    ${PROJECT_SOURCE_DIR}/src/LifecycleComponent.cpp)

target_link_libraries(${PROJECT_NAME} clproto)

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  include_directories(${PROJECT_SOURCE_DIR}/test/cpp/include)

  # add cpp tests
  file(GLOB_RECURSE TEST_CPP_SOURCES ${PROJECT_SOURCE_DIR}/test/ test_*.cpp)

  ament_add_gtest(test_modulo_components ${TEST_CPP_SOURCES})
  target_include_directories(test_modulo_components PRIVATE include)
  target_link_libraries(test_modulo_components ${PROJECT_NAME} state_representation clproto)
  target_compile_definitions(test_modulo_components PRIVATE TEST_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/")

  # prevent pluginlib from using boost
  target_compile_definitions(test_modulo_components PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

  # add python tests
  file(GLOB_RECURSE TEST_PYTHON_SOURCES ${PROJECT_SOURCE_DIR}/test/ test_*.py)

  foreach(TEST_PYTHON IN LISTS TEST_PYTHON_SOURCES)
    get_filename_component(TEST_FILENAME ${TEST_PYTHON} NAME_WE)
    ament_add_pytest_test(${TEST_FILENAME} ${TEST_PYTHON})
  endforeach()

endif()

ament_auto_package()
