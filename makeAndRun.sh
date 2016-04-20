cd ~/catkin_ws
catkin_make
cd -
echo $'\nRunning pplTracker\n'
rosrun people_tracker_denbyk pplTracker
