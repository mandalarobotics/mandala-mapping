cmake_minimum_required(VERSION 2.8.3)
project(sick_minimal_driver)

find_package(catkin REQUIRED)


find_package(catkin REQUIRED COMPONENTS  message_generation std_msgs)
include_directories(${catkin_INCLUDE_DIRS})



find_package(catkin REQUIRED COMPONENTS roscpp tf)

 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   sensor_msgs
 )

catkin_package(CATKIN_DEPENDS message_runtime  roscpp )


 add_executable(sick_minimal_driver src/lms_poller.cpp src/lms_mini_lib.cpp)


 target_link_libraries(sick_minimal_driver
   ${catkin_LIBRARIES}
)


## Mark executables and/or libraries for installation
 install(TARGETS sick_minimal_driver
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
