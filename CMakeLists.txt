cmake_minimum_required(VERSION 2.8.3)

project(people_tracker)

#orocos_kdl va a parte perchè non è un package catkin
find_package(orocos_kdl REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  roslib
  tf
  people_tracker_msg
  cv_bridge
)

catkin_package(
	INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs roslib tf sensor_msgs cv_bridge
  DEPENDS orocos_kdl OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/ni
  /usr/include/nite
  ${orocos_kdl_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  &{cv_bridge_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${cv_bridge_LIBRARY_DIRS})

#real stuff
add_executable(people_tracker src/people_tracker.cpp src/TrackingDenoiser.cpp)
add_dependencies(people_tracker people_tracker_msg_generate_messages_cpp)

target_link_libraries(people_tracker
  ${catkin_LIBRARIES}
	${OpenNI_LIBRARIES}
	${orocos_kdl_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(people_tracker OpenNI)


#just to test
#add_executable(testKalman src/testKalman.cpp)
#target_link_libraries(testKalman
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBS}
#)
