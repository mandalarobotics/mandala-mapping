cmake_minimum_required(VERSION 2.8.3)
project(m3dunit_base)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  m3d_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES m3d_driver
   CATKIN_DEPENDS roscpp tf m3d_msgs
   DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)



add_executable(m3d_driver_node src/encoder_node_li.cpp src/driverLib.cpp)
target_link_libraries(m3d_driver_node ${catkin_LIBRARIES})

 install(PROGRAMS
   scripts/joyToBool.py
   scripts/transformBroadcaster.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark executables and/or libraries for installation
 install(TARGETS m3d_driver_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
   launch/universal.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
