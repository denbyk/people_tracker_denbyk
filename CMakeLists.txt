cmake_minimum_required(VERSION 2.8.3)

project(people_tracker_denbyk)

#orocos_kdl va a parte perchè non è un package catkin
find_package(orocos_kdl REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  roslib
  tf
  people_tracker_denbyk_msg
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
add_executable(pplTracker src/PeopleTracker_denbyk.cpp)
add_dependencies(pplTracker people_tracker_denbyk_msg_generate_messages_cpp)

target_link_libraries(pplTracker
  ${catkin_LIBRARIES}
	${OpenNI_LIBRARIES}
	${orocos_kdl_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(pplTracker OpenNI)


#just to test
#add_executable(testKalman src/testKalman.cpp)
#target_link_libraries(testKalman
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBS}
#)
