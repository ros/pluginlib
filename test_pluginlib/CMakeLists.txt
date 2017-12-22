cmake_minimum_required(VERSION 3.5)
project(test_pluginlib)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(class_loader REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rcutils REQUIRED)
  find_package(tinyxml2_vendor REQUIRED)
  find_package(TinyXML2 REQUIRED)  # provided by tinyxml2 upstream, or tinyxml2_vendor

  add_library(test_plugins EXCLUDE_FROM_ALL SHARED test/test_plugins.cpp)
  target_include_directories(test_plugins PUBLIC
    include
    ${ament_index_cpp_INCLUDE_DIRS}
    ${class_loader_INCLUDE_DIRS}
    ${pluginlib_INCLUDE_DIRS}
    ${tinyxml2_INCLUDE_DIRS}
  )
  # place the library in an FHS compliant layout in the build space
  set(prefix "${CMAKE_CURRENT_BINARY_DIR}/prefix")
  set_target_properties(
    test_plugins PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${prefix}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${prefix}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${prefix}/bin"
  )
  # place the plugin manifest in an FHS compliant layout in the build space
  file(COPY test/test_plugins.xml DESTINATION "${prefix}/share/pluginlib/test")
  # create an entry in the ament index for the exported plugins
  file(WRITE "${prefix}/share/ament_index/resource_index/pluginlib__plugin/pluginlib"
    "\${prefix}/test/test_plugins.xml\n")

  ament_add_gtest(${PROJECT_NAME}_utest
    test/utest.cpp
    APPEND_LIBRARY_DIRS "$<TARGET_FILE_DIR:test_plugins>"
  )
  if(TARGET ${PROJECT_NAME}_utest)
    target_include_directories(${PROJECT_NAME}_utest PUBLIC 
      include
      ${ament_index_cpp_INCLUDE_DIRS}
      ${class_loader_INCLUDE_DIRS}
      ${pluginlib_INCLUDE_DIRS}
      ${rcutils_INCLUDE_DIRS}
    )
    target_link_libraries(${PROJECT_NAME}_utest
      ${ament_index_cpp_LIBRARIES}
      ${class_loader_LIBRARIES}
      ${TinyXML2_LIBRARIES}
      ${rcutils_LIBRARIES}
      ${PROJECT_NAME_LIBRARIES}
    )
    if(CMAKE_COMPILER_IS_GNUCXX)
      target_link_libraries(${PROJECT_NAME}_utest "stdc++fs")
    endif()
    add_dependencies(${PROJECT_NAME}_utest
      test_plugins)
  endif()

  # include(CheckCXXCompilerFlag)
  # check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
  # if(COMPILER_SUPPORTS_CXX11)
  #   catkin_add_gtest(${PROJECT_NAME}_unique_ptr_test test/unique_ptr_test.cpp)
  #   if(TARGET ${PROJECT_NAME}_unique_ptr_test)
  #     target_link_libraries(${PROJECT_NAME}_unique_ptr_test
  #       ${tinyxml2_LIBRARIES} ${catkin_LIBRARIES})
  #     set_target_properties(${PROJECT_NAME}_unique_ptr_test
  #       PROPERTIES COMPILE_FLAGS -std=c++11 LINK_FLAGS -std=c++11)
  #     add_dependencies(${PROJECT_NAME}_unique_ptr_test test_plugins)
  #   endif()
  # endif()
endif()

ament_package()
