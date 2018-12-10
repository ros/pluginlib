cmake_minimum_required(VERSION 3.5)
project(pluginlib)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(rcutils REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)  # provided by tinyxml2 upstream, or tinyxml2_vendor

ament_export_dependencies(ament_index_cpp class_loader rcutils tinyxml2_vendor TinyXML2)
ament_export_include_directories(include)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(prefix "${CMAKE_CURRENT_BINARY_DIR}/prefix")
  file(COPY test/test_plugins.xml DESTINATION "${prefix}/share/pluginlib/test")
  file(WRITE "${prefix}/share/ament_index/resource_index/pluginlib__plugin/pluginlib"
    "\${prefix}/test/test_plugins.xml\n")

  add_library(test_plugins SHARED ./test/test_plugins.cpp)
  target_link_libraries(test_plugins ${class_loader_LIBRARIES})
  target_compile_definitions(test_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
  target_compile_definitions(test_plugins PRIVATE "TEST_PLUGINLIB_FIXTURE_BUILDING_LIBRARY")
  target_include_directories(test_plugins
      PUBLIC
      include
      test/include
      ${ament_index_cpp_INCLUDE_DIRS}
      ${class_loader_INCLUDE_DIRS}
      ${rcutils_INCLUDE_DIRS}
      ${TinyXML2_INCLUDE_DIRS}
    )
  set_target_properties(
    test_plugins PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${prefix}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${prefix}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${prefix}/bin"
  )

  ament_add_gtest(${PROJECT_NAME}_unique_ptr_test
    test/unique_ptr_test.cpp
    APPEND_LIBRARY_DIRS "$<TARGET_FILE_DIR:test_plugins>"
  )
  if(TARGET ${PROJECT_NAME}_unique_ptr_test)
    target_include_directories(${PROJECT_NAME}_unique_ptr_test
      PUBLIC
      include
      test/include
      ${ament_index_cpp_INCLUDE_DIRS}
      ${class_loader_INCLUDE_DIRS}
      ${rcutils_INCLUDE_DIRS}
      ${TinyXML2_INCLUDE_DIRS}
    )
    target_compile_definitions(${PROJECT_NAME}_unique_ptr_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
    target_link_libraries(${PROJECT_NAME}_unique_ptr_test
      ${ament_index_cpp_LIBRARIES}
      ${class_loader_LIBRARIES}
      ${rcutils_LIBRARIES}
      ${TinyXML2_LIBRARIES})
    if(CMAKE_COMPILER_IS_GNUCXX)
      target_link_libraries(${PROJECT_NAME}_unique_ptr_test stdc++fs)
    endif()
    add_dependencies(${PROJECT_NAME}_unique_ptr_test test_plugins)
  endif()

  ament_add_gtest(${PROJECT_NAME}_utest
    test/utest.cpp
    APPEND_LIBRARY_DIRS "$<TARGET_FILE_DIR:test_plugins>"
  )
  if(TARGET ${PROJECT_NAME}_utest)
    target_include_directories(${PROJECT_NAME}_utest
      PUBLIC
      include
      test/include
      ${ament_index_cpp_INCLUDE_DIRS}
      ${class_loader_INCLUDE_DIRS}
      ${rcutils_INCLUDE_DIRS}
      ${TinyXML2_INCLUDE_DIRS}
    )
    target_compile_definitions(${PROJECT_NAME}_utest PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
    target_link_libraries(${PROJECT_NAME}_utest
      ${ament_index_cpp_LIBRARIES}
      ${class_loader_LIBRARIES}
      ${rcutils_LIBRARIES}
      ${TinyXML2_LIBRARIES})
    if(CMAKE_COMPILER_IS_GNUCXX)
      target_link_libraries(${PROJECT_NAME}_utest stdc++fs)
    endif()
    add_dependencies(${PROJECT_NAME}_utest test_plugins)
  endif()

endif()

ament_package(
  CONFIG_EXTRAS "pluginlib-extras.cmake"
)
