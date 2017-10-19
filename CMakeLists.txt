cmake_minimum_required(VERSION 2.8.3)
project(pluginlib)

find_package(catkin REQUIRED COMPONENTS class_loader rosconsole roslib cmake_modules)
find_package(Boost REQUIRED)  # should be removed when deprecated createInstance is removed
find_package(TinyXML2 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS class_loader rosconsole roslib
  DEPENDS
    Boost  # should be removed when deprecated createInstance is removed
    TinyXML2
)

install(DIRECTORY include/pluginlib/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML2_INCLUDE_DIRS})

  add_library(test_plugins EXCLUDE_FROM_ALL SHARED test/test_plugins.cpp)

  catkin_add_gtest(${PROJECT_NAME}_utest test/utest.cpp)
  if(TARGET ${PROJECT_NAME}_utest)
    target_link_libraries(${PROJECT_NAME}_utest ${TinyXML2_LIBRARIES} ${catkin_LIBRARIES})
    add_dependencies(${PROJECT_NAME}_utest test_plugins)
  endif()

  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  if(COMPILER_SUPPORTS_CXX11)
    catkin_add_gtest(${PROJECT_NAME}_unique_ptr_test test/unique_ptr_test.cpp)
    if(TARGET ${PROJECT_NAME}_unique_ptr_test)
      target_link_libraries(${PROJECT_NAME}_unique_ptr_test ${TinyXML2_LIBRARIES} ${catkin_LIBRARIES})
      set_target_properties(${PROJECT_NAME}_unique_ptr_test PROPERTIES COMPILE_FLAGS -std=c++11 LINK_FLAGS -std=c++11)
      add_dependencies(${PROJECT_NAME}_unique_ptr_test test_plugins)
    endif()
  endif()

endif()
