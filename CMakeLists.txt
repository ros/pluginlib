cmake_minimum_required(VERSION 2.8.3)
project(pluginlib)

find_package(catkin REQUIRED COMPONENTS class_loader rosconsole roslib)
find_package(Boost REQUIRED COMPONENTS filesystem system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES boost_fs_wrapper tinyxml
  DEPENDS class_loader rosconsole roslib
  DEPENDS Boost COMPONENTS filesystem system
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(boost_fs_wrapper SHARED src/boost_fs_wrapper.cpp)
target_link_libraries(boost_fs_wrapper ${Boost_LIBRARIES}) 

install(TARGETS boost_fs_wrapper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/pluginlib/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_library(test_plugins EXCLUDE_FROM_ALL SHARED test/test_plugins.cpp)
target_link_libraries(test_plugins boost_fs_wrapper)
catkin_add_gtest(${PROJECT_NAME}_utest test/utest.cpp)
target_link_libraries(${PROJECT_NAME}_utest boost_fs_wrapper tinyxml ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_utest test_plugins)
