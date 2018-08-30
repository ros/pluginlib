cmake_minimum_required(VERSION 2.8.3)
project(pluginlib)

find_package(catkin REQUIRED COMPONENTS class_loader rosconsole roslib cmake_modules)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(TinyXML2 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS class_loader rosconsole roslib
  DEPENDS Boost TinyXML2
)

if(CATKIN_ENABLE_TESTING)
  include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${TinyXML2_INCLUDE_DIRS})

  add_library(test_plugins EXCLUDE_FROM_ALL SHARED test/test_plugins.cpp)
  target_link_libraries(test_plugins ${class_loader_LIBRARIES})
  if(WIN32)
    # On Windows, default library runtime output set to CATKIN_GLOBAL_BIN_DESTINATION,
    # change it back to CATKIN_PACKAGE_LIB_DESTINATION to match the library path described in plugin description file
    set_target_properties(test_plugins PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
  endif()

  catkin_add_gtest(${PROJECT_NAME}_utest test/utest.cpp)
  if(TARGET ${PROJECT_NAME}_utest)
    target_link_libraries(${PROJECT_NAME}_utest ${TinyXML2_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    add_dependencies(${PROJECT_NAME}_utest test_plugins)
  endif()

  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
  if(COMPILER_SUPPORTS_CXX11)
    catkin_add_gtest(${PROJECT_NAME}_unique_ptr_test test/unique_ptr_test.cpp)
    if(TARGET ${PROJECT_NAME}_unique_ptr_test)
      target_link_libraries(${PROJECT_NAME}_unique_ptr_test ${TinyXML2_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
      set_target_properties(${PROJECT_NAME}_unique_ptr_test PROPERTIES COMPILE_FLAGS -std=c++11 LINK_FLAGS -std=c++11)
      add_dependencies(${PROJECT_NAME}_unique_ptr_test test_plugins)
    endif()
  endif()

endif()

install(DIRECTORY include/pluginlib/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(PROGRAMS scripts/pluginlib_headers_migration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
