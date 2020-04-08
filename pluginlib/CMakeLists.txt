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
find_package(rcpputils REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)  # provided by tinyxml2 upstream, or tinyxml2_vendor

ament_export_dependencies(ament_index_cpp class_loader rcutils rcpputils tinyxml2_vendor TinyXML2)
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

  # NOTE this block is also defined in pluginlib-extras.cmake
  # changes here need to be duplicated there
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_FLAGS MATCHES "-stdlib=libc\\+\\+")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.0)
      # Before LLVM 7.0, filesystem is part of experimental
      set(FILESYSTEM_LIB c++experimental)
    elseif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
      # Before LLVM 9.0 you have to manually link the fs library
      set(FILESYSTEM_LIB c++fs)
    else()
      # Starting at LLVM 9.0 filesystem is built in
      set(FILESYSTEM_LIB)
    endif()
  else()
    set(FILESYSTEM_LIB stdc++fs)
  endif()

  set(prefix "${CMAKE_CURRENT_BINARY_DIR}/prefix")

  # Write the necessary files for ament to detect a package.
  file(COPY test/test_plugins.xml DESTINATION "${prefix}/share/test_pluginlib/")
  configure_file(test/test_package.xml "${prefix}/share/test_pluginlib/package.xml" COPYONLY)

  file(WRITE "${prefix}/share/ament_index/resource_index/test_pluginlib__pluginlib__plugin/pluginlib"
    "share/test_pluginlib/test_plugins.xml\n")
  file(WRITE "${prefix}/share/ament_index/resource_index/packages/test_pluginlib" "")

  include_directories(include test/include)

  add_library(test_plugins SHARED ./test/test_plugins.cpp)
  target_compile_definitions(test_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
  target_compile_definitions(test_plugins PRIVATE "TEST_PLUGINLIB_FIXTURE_BUILDING_LIBRARY")
  ament_target_dependencies(
    test_plugins
    class_loader
  )

  # Use a generator expression so that the specified folder is used directly, without any
  # configuration-dependent suffix.
  #
  # See https://cmake.org/cmake/help/v3.8/prop_tgt/LIBRARY_OUTPUT_DIRECTORY.html
  # Ref: https://stackoverflow.com/a/45871818
  set_target_properties(
    test_plugins PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${prefix}/lib/$<$<CONFIG:Debug>:>"
    LIBRARY_OUTPUT_DIRECTORY "${prefix}/lib/$<$<CONFIG:Debug>:>"
    RUNTIME_OUTPUT_DIRECTORY "${prefix}/bin/$<$<CONFIG:Debug>:>"
  )

  ament_add_gtest(${PROJECT_NAME}_unique_ptr_test
    test/unique_ptr_test.cpp
    APPEND_LIBRARY_DIRS "$<TARGET_FILE_DIR:test_plugins>"
    ENV AMENT_PREFIX_PATH=${prefix}
  )
  if(TARGET ${PROJECT_NAME}_unique_ptr_test)
    ament_target_dependencies(
      ${PROJECT_NAME}_unique_ptr_test
      class_loader
      ament_index_cpp
      rcpputils
      rcutils
      TinyXML2
    )
    target_compile_definitions(${PROJECT_NAME}_unique_ptr_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

    if(UNIX AND NOT APPLE)
      target_link_libraries(${PROJECT_NAME}_unique_ptr_test ${FILESYSTEM_LIB})
    endif()
    add_dependencies(${PROJECT_NAME}_unique_ptr_test test_plugins)
  endif()

  ament_add_gtest(${PROJECT_NAME}_utest
    test/utest.cpp
    APPEND_LIBRARY_DIRS "$<TARGET_FILE_DIR:test_plugins>"
    ENV AMENT_PREFIX_PATH=${prefix}
  )
  if(TARGET ${PROJECT_NAME}_utest)
    ament_target_dependencies(
      ${PROJECT_NAME}_utest
      class_loader
      ament_index_cpp
      rcpputils
      rcutils
      TinyXML2
    )
    target_compile_definitions(${PROJECT_NAME}_utest PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

    if(UNIX AND NOT APPLE)
      target_link_libraries(${PROJECT_NAME}_utest ${FILESYSTEM_LIB})
    endif()
    add_dependencies(${PROJECT_NAME}_utest test_plugins)
  endif()

endif()

ament_package(
  CONFIG_EXTRAS "pluginlib-extras.cmake"
)
