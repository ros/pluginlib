cmake_minimum_required(VERSION 3.20)
project(pluginlib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcpputils REQUIRED)
find_package(TinyXML2 REQUIRED)

# Everything that does not depend on the plugin base class T is compiled once here,
# rather than re-instantiated in every consumer for every ClassLoader<T>. Only the
# genuinely T-dependent members remain in the headers.
add_library(${PROJECT_NAME} SHARED
  src/class_desc.cpp
  src/class_loader_base.cpp
  src/class_loader_impl.cpp
  src/exceptions.cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE "PLUGINLIB_BUILDING_LIBRARY")
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  ament_cmake_ros_core::ament_ros_defaults
  ament_index_cpp::ament_index_cpp
  class_loader::class_loader
  rcutils::rcutils
  rcpputils::rcpputils)
# tinyxml2 is only used by the XML parsing in class_loader_impl.cpp; it is no longer
# reachable from any public header, so consumers do not need it.
target_link_libraries(${PROJECT_NAME} PRIVATE
  tinyxml2::tinyxml2)

add_executable(list_plugins src/list_plugins.cpp)
target_include_directories(list_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(list_plugins
  ${PROJECT_NAME}
)
install(TARGETS list_plugins
        DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  ament_cmake_ros_core ament_index_cpp class_loader rcutils rcpputils TinyXML2)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  add_library(test_plugins SHARED ./test/test_plugins.cpp)
  target_compile_definitions(test_plugins PRIVATE "TEST_PLUGINLIB_FIXTURE_BUILDING_LIBRARY")
  target_include_directories(test_plugins PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>")
  target_link_libraries(test_plugins ${PROJECT_NAME})

  include("cmake/pluginlib_enable_plugin_testing.cmake")
  pluginlib_enable_plugin_testing(
    CMAKE_TARGET_VAR mock_install_target
    AMENT_PREFIX_PATH_VAR mock_install_path
    PACKAGE_NAME "test_pluginlib"
    PACKAGE_XML "test/test_package.xml"
    PLUGIN_CATEGORY "test_pluginlib"
    PLUGIN_DESCRIPTIONS "test/test_plugins.xml"
    PLUGIN_LIBRARIES test_plugins
  )

  ament_add_gtest(${PROJECT_NAME}_unique_ptr_test
    test/unique_ptr_test.cpp
    APPEND_LIBRARY_DIRS "$<TARGET_FILE_DIR:test_plugins>"
    APPEND_ENV AMENT_PREFIX_PATH=${mock_install_path}
  )
  if(TARGET ${PROJECT_NAME}_unique_ptr_test)
    target_link_libraries(${PROJECT_NAME}_unique_ptr_test ${PROJECT_NAME})
    target_include_directories(${PROJECT_NAME}_unique_ptr_test PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>")
    add_dependencies(${PROJECT_NAME}_unique_ptr_test "${mock_install_target}")
  endif()

  ament_add_gtest(${PROJECT_NAME}_utest
    test/utest.cpp
    APPEND_ENV AMENT_PREFIX_PATH=${mock_install_path}
  )
  if(TARGET ${PROJECT_NAME}_utest)
    # Used, but not linked to test ${PROJECT_NAME}'s exports:
    #   rcutils::rcutils
    target_link_libraries(${PROJECT_NAME}_utest ${PROJECT_NAME})
    add_dependencies(${PROJECT_NAME}_utest "${mock_install_target}")
    target_include_directories(${PROJECT_NAME}_utest PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>")
  endif()

endif()

ament_package(
  CONFIG_EXTRAS "pluginlib-extras.cmake"
)
