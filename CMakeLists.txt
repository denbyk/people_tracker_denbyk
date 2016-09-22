cmake_minimum_required(VERSION 2.8.3)

project(people_tracker_denbyk)

#orocos_kdl va a parte perchè non è un package catkin
find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
  tf
)
catkin_package(
	INCLUDE_DIRS include
#  orocos_kdl
#  CATKIN_DEPENDS roscpp std_msgs roslib tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/ni
  /usr/include/nite
)

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_executable(pplTracker src/PeopleTracker_denbyk.cpp)

target_link_libraries(pplTracker
  ${catkin_LIBRARIES}
	${OpenNI_LIBRARIES}
	${orocos_kdl_LIBRARIES}
)

target_link_libraries(pplTracker OpenNI)
