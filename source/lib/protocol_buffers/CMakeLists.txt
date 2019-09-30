cmake_minimum_required(VERSION 3.9)

# Options. Turn on with 'cmake -Druntests=ON'.
option(runtests "Build all tests." OFF) # Makes boolean 'test' available.

project(protocol_buffers)

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

find_package(Protobuf REQUIRED)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/modifiedprotobuf.cmake)

file(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")

modified_protobuf_generate_cpp(generated_files/${PROJECT_NAME} PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/generated_files
  ${PROTOBUF_INCLUDE_DIRS}
)

set(CORE_SOURCES
  src/Communication.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${CORE_SOURCES}
  ${PROTO_SRCS}
)

target_link_libraries(${PROJECT_NAME}
  ${PROTOBUF_LIBRARIES}
  zmq
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated_files/ include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if (runtests)
    if (APPLE)
      add_definitions(-DGTEST_USE_OWN_TR1_TUPLE)
      add_definitions(-D__GLIBCXX__)
    endif (APPLE)

    add_subdirectory(lib/googletest)

    enable_testing()

    # Include the gtest library. gtest_SOURCE_DIR is available due to
    # 'project(gtest)' above.
    include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR} dynamical_systems/include)

    add_executable(runTestPublisher tests/testPublisher.cpp)
    target_link_libraries(runTestPublisher
      gtest 
      gtest_main
      ${PROJECT_NAME}
      state_representation
      ${PROTOBUF_LIBRARIES}
    )
    add_test(NAME runTestPublisher COMMAND runTestPublisher)
endif()