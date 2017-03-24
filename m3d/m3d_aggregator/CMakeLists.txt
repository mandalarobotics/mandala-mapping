cmake_minimum_required(VERSION 2.8.3)
project(m3d_aggregator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#SET(CMAKE_BUILD_TYPE Release)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  pcl_ros
  tf_conversions
  m3d_msgs
)
find_package(PCL 1.5 REQUIRED)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES m3d_driver
   CATKIN_DEPENDS roscpp tf pcl_ros tf_conversion m3d_msgs
   DEPENDS system_lib Boost PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



## Declare a cpp executable
add_executable(m3d_aggregator src/m3d_aggregator.cpp )

target_link_libraries(m3d_aggregator ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS m3d_aggregator
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
