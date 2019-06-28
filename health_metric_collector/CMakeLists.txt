cmake_minimum_required(VERSION 2.8.3)
project(health_metric_collector)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ros_monitoring_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  aws_common
  aws_ros1_common
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(ALL_SRC_FILES src/cpu_data.cpp src/cpu_stats.cpp src/metric_manager.cpp src/cpu_metric_collector.cpp src/sys_info_collector.cpp)
add_library(collector_lib ${ALL_SRC_FILES})

add_executable(health_metric_collector src/collector.cpp)
target_link_libraries(health_metric_collector ${catkin_LIBRARIES} collector_lib)

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

install(TARGETS collector_lib DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS health_metric_collector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GMock QUIET)
  if(GMOCK_FOUND)
    add_rostest_gmock(test_health_metric_collector
      test/test_health_metric_collector.test
      test/health_metric_collector_test.cpp
    )
    target_include_directories(test_health_metric_collector
      PRIVATE include
      ${catkin_INCLUDE_DIRS}
      ${GMOCK_INCLUDE_DIRS}
    )
    target_link_libraries(test_health_metric_collector
      collector_lib
      ${catkin_LIBRARIES}
      ${GMOCK_BOTH_LIBRARIES}
    )
  else()
    include_directories(/usr/include/gmock /usr/src/gmock)
    add_library(${PROJECT_NAME}_libgmock SHARED /usr/src/gmock/src/gmock-all.cc)

    add_rostest_gtest(test_health_metric_collector
      test/test_health_metric_collector.test
      test/health_metric_collector_test.cpp
    )
    target_include_directories(test_health_metric_collector
      PRIVATE include
      ${catkin_INCLUDE_DIRS}
    )
    target_link_libraries(test_health_metric_collector
      collector_lib
      ${catkin_LIBRARIES}
      ${PROJECT_NAME}_libgmock
    )
  endif()
endif()
