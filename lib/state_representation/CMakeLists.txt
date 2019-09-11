cmake_minimum_required(VERSION 3.9)

# Options. Turn on with 'cmake -Druntests=ON'.
option(runtests "Build all tests." OFF) # Makes boolean 'test' available.

project(state_representation)

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

find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(
 	include
 	${Eigen3_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
)

set(CORE_SOURCES
  src/State.cpp
	src/Cartesian/CartesianPose.cpp
  src/Cartesian/CartesianVelocity.cpp
	src/Cartesian/CartesianState.cpp
  src/Joint/JointState.cpp
  src/Joint/JointPositions.cpp
  src/Joint/JointVelocities.cpp
  src/DualQuaternion/DualQuaternionState.cpp
  src/DualQuaternion/DualQuaternionPose.cpp
  src/DualQuaternion/DualQuaternionTwist.cpp
  src/Robot/JacobianMatrix.cpp
)

add_library(${PROJECT_NAME} SHARED 
  ${CORE_SOURCES}
)

target_link_libraries(${PROJECT_NAME}
  protocol_buffers
)

install(DIRECTORY include/
  	DESTINATION include
)

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
  	include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

  	add_executable(runTestCartesianState tests/testCartesianState.cpp)
  	target_link_libraries(runTestCartesianState
  		gtest 
  		gtest_main
  		${PROJECT_NAME}
      zmq
      ${PROTOBUF_LIBRARIES}
  	)
	  add_test(NAME runTestCartesianState COMMAND runTestCartesianState)

    add_executable(runTestJointState tests/testJointState.cpp)
    target_link_libraries(runTestJointState
      gtest 
      gtest_main
      ${PROJECT_NAME}
      zmq
      ${PROTOBUF_LIBRARIES}
    )
    add_test(NAME runTestJointState COMMAND runTestJointState)

    add_executable(runTestDualQuaternionState tests/testDualQuaternionState.cpp)
    target_link_libraries(runTestDualQuaternionState
      gtest 
      gtest_main
      ${PROJECT_NAME}
    )
    add_test(NAME runTestDualQuaternionState COMMAND runTestDualQuaternionState)

    add_executable(runTestJacobianMatrix tests/testJacobianMatrix.cpp)
    target_link_libraries(runTestJacobianMatrix
      gtest 
      gtest_main
      ${PROJECT_NAME}
    )
    add_test(NAME runTestJacobianMatrix COMMAND runTestJacobianMatrix)

endif()
