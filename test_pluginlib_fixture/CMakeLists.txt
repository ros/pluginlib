cmake_minimum_required(VERSION 3.5)
project(test_pluginlib_fixture)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_index_cpp REQUIRED)
  find_package(class_loader REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rcutils REQUIRED)
  find_package(tinyxml2_vendor REQUIRED)
  find_package(TinyXML2 REQUIRED)  # provided by tinyxml2 upstream, or tinyxml2_vendor

  add_library(${PROJECT_NAME} SHARED src/test_plugins.cpp)
  target_include_directories(${PROJECT_NAME} PUBLIC include)

  target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
  target_compile_definitions(${PROJECT_NAME} PRIVATE "TEST_PLUGINLIB_FIXTURE_BUILDING_LIBRARY")

  ament_target_dependencies(${PROJECT_NAME}
    "pluginlib"
    "rcutils"
  )

  pluginlib_export_plugin_description_file(${PROJECT_NAME} test_plugins.xml)

  install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(
    DIRECTORY include/
    DESTINATION include
  )

  ament_export_include_directories(include)
  ament_export_libraries(${PROJECT_NAME})
  ament_export_dependencies(pluginlib rcutils)
endif()

ament_package()
