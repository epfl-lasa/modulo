cmake_minimum_required(VERSION 3.9)

# Options. Turn on with 'cmake -Druntests=ON'.
option(runtests "Build all tests." OFF) # Makes boolean 'test' available.

project(dynamical_systems)

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

include_directories(
  	include
    state_representation/include
  	${Eigen3_INCLUDE_DIRS}
)

set(CORE_SOURCES
  src/DynamicalSystem.cpp
  src/Circular.cpp
  src/Linear.cpp
)

add_library(${PROJECT_NAME} SHARED 
  ${CORE_SOURCES}
)

install(DIRECTORY include/
  	DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  	ARCHIVE DESTINATION lib
  	LIBRARY DESTINATION lib
  	RUNTIME DESTINATION bin)

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

  	add_executable(runTestLinear tests/testLinear.cpp)
  	target_link_libraries(runTestLinear
  		gtest 
  		gtest_main
  		${PROJECT_NAME}
      state_representation
  	)
	  add_test(NAME runTestLinear COMMAND runTestLinear)

    add_executable(runTestCircular tests/testCircular.cpp)
    target_link_libraries(runTestCircular
      gtest 
      gtest_main
      ${PROJECT_NAME}
      state_representation
    )
    add_test(NAME runTestCircular COMMAND runTestCircular)

endif()
