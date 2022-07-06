cmake_minimum_required(VERSION 3.15)
project(modulo_core)

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

find_package(control_libraries 6.0.5 REQUIRED COMPONENTS state_representation)
find_package(clproto 6.0.0 REQUIRED)

include_directories(include)

ament_auto_add_library(modulo_core SHARED
    ${PROJECT_SOURCE_DIR}/src/communication/MessagePair.cpp
    ${PROJECT_SOURCE_DIR}/src/communication/MessagePairInterface.cpp
    ${PROJECT_SOURCE_DIR}/src/communication/PublisherInterface.cpp
    ${PROJECT_SOURCE_DIR}/src/communication/SubscriptionHandler.cpp
    ${PROJECT_SOURCE_DIR}/src/communication/SubscriptionInterface.cpp
    ${PROJECT_SOURCE_DIR}/src/translators/parameter_translators.cpp
    ${PROJECT_SOURCE_DIR}/src/translators/message_readers.cpp
    ${PROJECT_SOURCE_DIR}/src/translators/message_writers.cpp)

# install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # run cpp tests
  file(GLOB_RECURSE TEST_CPP_SOURCES ${PROJECT_SOURCE_DIR}/test/test_*.cpp)

  ament_add_gtest(test_modulo_core ${TEST_CPP_SOURCES})
  target_include_directories(test_modulo_core PRIVATE include)
  target_link_libraries(test_modulo_core modulo_core state_representation clproto)

  # prevent pluginlib from using boost
  target_compile_definitions(test_modulo_core PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

  # run python tests
  file(GLOB_RECURSE TEST_PYTHON_SOURCES ${PROJECT_SOURCE_DIR}/test/test_*.py)

  foreach(TEST_PYTHON IN LISTS TEST_PYTHON_SOURCES)
    get_filename_component(TEST_FILENAME ${TEST_PYTHON} NAME_WE)
    ament_add_pytest_test(${TEST_FILENAME} ${TEST_PYTHON})
  endforeach()

endif()

ament_auto_package()
