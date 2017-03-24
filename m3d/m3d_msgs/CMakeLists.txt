cmake_minimum_required(VERSION 2.8.3)
project(m3d_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)



add_service_files(
   FILES
   calibration.srv
 )
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
 )
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES m3d_driver
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)
