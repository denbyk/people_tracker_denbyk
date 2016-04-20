cmake_minimum_required(VERSION 2.8.3)
project(people_tracker_denbyk)
find_package(catkin REQUIRED roscpp std_msgs)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pplTracker src/PeopleTracker_denbyk.cpp)
target_link_libraries(pplTracker ${catkin_LIBRARIES})
#add_dependencies(talker beginner_tutorials_generate_messages_cpp)
