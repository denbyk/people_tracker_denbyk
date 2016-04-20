#include <ros/ros.h>
#include <ros/package.h>
#include <string>


/*
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCppWrapper.h>
#include <XnCodecIDs.h>
*/


int main(int argc, char **argv){
  ros::init(argc, argv, "pplTracker");
  ros::NodeHandle n;

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    std::cout << "ciao ";
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 1;
}
