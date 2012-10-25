cmake_minimum_required(VERSION 2.8.3)
project(pluginlib)

find_package(catkin REQUIRED COMPONENTS
  class_loader cpp_common rosconsole roslib rostime
)
find_package(Boost REQUIRED COMPONENTS filesystem system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES boost_fs_wrapper tinyxml
  DEPENDS class_loader rosconsole
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(boost_fs_wrapper SHARED src/boost_fs_wrapper.cpp)
target_link_libraries(boost_fs_wrapper ${Boost_LIBRARIES}) 

add_library(test_plugins EXCLUDE_FROM_ALL SHARED test/test_plugins.cpp)
target_link_libraries(test_plugins boost_fs_wrapper)
catkin_add_gtest(utest test/utest.cpp)
target_link_libraries(utest boost_fs_wrapper tinyxml ${catkin_LIBRARIES})
add_dependencies(utest test_plugins)

install(TARGETS boost_fs_wrapper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/pluginlib/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

