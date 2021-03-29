cmake_minimum_required(VERSION 2.8.3)
project(bebop2)

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
set(BEBOP2_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../../AirSim/)
set(CMAKE_CXX_STANDARD 11) 

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	sensor_msgs
	nav_msgs
	roslib
	cv_bridge
	image_transport	
	message_generation
)

add_message_files(
  FILES
  YawCmd.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES airsim_ros
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(keyboard src/keyboard.cpp)
add_dependencies(keyboard ${catkin_EXPORTED_TARGETS})

add_library(retinex src/retinex.cpp)
add_dependencies(retinex ${catkin_EXPORTED_TARGETS})

add_library(gps_controller src/gps_controller.cpp)
add_dependencies(gps_controller ${catkin_EXPORTED_TARGETS})

add_executable(image_bebop2_node src/image_bebop2_node.cpp)
target_link_libraries(image_bebop2_node ${catkin_LIBRARIES} ${OpenCV_LIBS} retinex)

add_executable(gps_bebop2_node src/gps_bebop2_node.cpp)
target_link_libraries(gps_bebop2_node ${catkin_LIBRARIES} keyboard)

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} keyboard gps_controller)
