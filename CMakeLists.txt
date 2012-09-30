cmake_minimum_required(VERSION 2.8)
project(pluginlib)

find_package(catkin REQUIRED COMPONENTS
  cpp_common rosconsole roslib rostime
)
find_package(Boost REQUIRED COMPONENTS filesystem system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES poco_lite boost_fs_wrapper tinyxml
  DEPENDS rosconsole
)

include_directories(include ${catkin_INCLUDE_DIRS})

set(${PROJECT_NAME}_SRCS
  src/ASCIIEncoding.cpp
  src/AtomicCounter.cpp
  src/Bugcheck.cpp
  src/Debugger.cpp
  src/DirectoryIterator.cpp
  src/Environment.cpp
  src/Exception.cpp
  src/File.cpp
  src/Latin1Encoding.cpp
  src/Latin9Encoding.cpp
  src/Manifest.cpp
  src/Mutex.cpp
  src/Path.cpp
  src/RWLock.cpp
  src/SharedLibrary.cpp
  src/StringTokenizer.cpp
  src/TextConverter.cpp
  src/TextEncoding.cpp
  src/TextIterator.cpp
  src/Timestamp.cpp
  src/UnicodeConverter.cpp
  src/UTF8Encoding.cpp
  src/UTF16Encoding.cpp
  src/Windows1252Encoding.cpp
)
add_library(poco_lite SHARED ${${PROJECT_NAME}_SRCS})
target_link_libraries(poco_lite dl tinyxml)

add_library(boost_fs_wrapper SHARED src/boost_fs_wrapper.cpp)
target_link_libraries(boost_fs_wrapper ${Boost_LIBRARIES}) 

add_library(test_plugins SHARED test/test_plugins.cpp)
target_link_libraries(test_plugins boost_fs_wrapper poco_lite)
catkin_add_gtest(utest test/utest.cpp)
target_link_libraries(utest boost_fs_wrapper poco_lite ${catkin_LIBRARIES})

install(FILES cmake/hide_plugin_symbols.cmake
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
install(TARGETS boost_fs_wrapper poco_lite
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")
