cmake_minimum_required(VERSION 3.5)
project(test_pluginlib)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  find_package(pluginlib REQUIRED)
  find_package(rcutils REQUIRED)
  find_package(test_pluginlib_fixture REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_unique_ptr_test
    test/unique_ptr_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_unique_ptr_test)
    ament_target_dependencies(${PROJECT_NAME}_unique_ptr_test
      pluginlib rcutils test_pluginlib_fixture)
    target_compile_definitions(${PROJECT_NAME}_unique_ptr_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
  endif()

  ament_add_gtest(${PROJECT_NAME}_utest
    test/utest.cpp
  )
  if(TARGET ${PROJECT_NAME}_utest)
    ament_target_dependencies(${PROJECT_NAME}_utest
      pluginlib rcutils test_pluginlib_fixture)
    target_compile_definitions(${PROJECT_NAME}_utest PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
  endif()
endif()
