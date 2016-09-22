#include <ros/ros.h>
#include <cob_perception_msgs/DetectionArray.h>
#include <std_msgs/String.h>

void userToTrackIdentityCallback(std_msgs::String);
void userToTrackFaceCallback(cob_perception_msgs::DetectionArray);

std::string user_to_track;
bool got_user = false;
bool auto_engage = false;

std::map<std::string, int> faces_counter;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "identity_setter");

	ros::NodeHandle nh;

	std::string face_topic("/cob_people_detection/face_recognizer/face_recognitions");
	std::string identity_topic("/hostess_skeleton_tracker/user_to_track");

	nh.getParam("face_topic", face_topic);
	nh.getParam("identity_topic", identity_topic);

	ros::Subscriber identitySub;

	if(!auto_engage)
	{
		identitySub = nh.subscribe(identity_topic, 1, userToTrackIdentityCallback);
	}
	else
	{
		identitySub = nh.subscribe(face_topic, 1, userToTrackFaceCallback);
	}

	ros::Rate rate(10.0);

	while(!got_user)
	{
		ros::spinOnce();

		rate.sleep();
	}

	identitySub.shutdown();

	ros::param::set("user_to_track", user_to_track);

	ros::shutdown();

	return EXIT_SUCCESS;
}

void userToTrackIdentityCallback(std_msgs::String msg)
{
	user_to_track = msg.data;
	got_user = true;
}

void userToTrackFaceCallback(cob_perception_msgs::DetectionArray msg)
{
	std::vector<cob_perception_msgs::Detection> identities = msg.detections;

	for(int i = 0; i < identities.size(); i++)
	{
		if(identities[i].detector == "face" && identities[i].label != "Unknown")
		{
			faces_counter[identities[i].label]++;
		}
	}

	for(std::map<std::string, int>::iterator iter = faces_counter.begin(); iter != faces_counter.end(); ++iter)
	{
		if(iter->second >= 100)
		{
			user_to_track = iter->first;
			got_user = true;
		}
	}
}
