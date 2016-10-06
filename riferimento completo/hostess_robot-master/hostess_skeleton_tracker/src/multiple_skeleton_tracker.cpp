#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <kdl/frames.hpp>
#include <sstream>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <vector>
#include <map>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#define MAX_USERS 6

std::string genericUserCalibrationFileName;
std::vector<std::string> joint_names{"head", "neck", "torso", "waist", "left_collar", "left_shoulder", "left_elbow", "left_wrist", "left_hand", "left_fingertip", "right_collar", "right_shoulder", "right_elbow", "right_wrist", "right_hand", "right_fingertip", "left_hip", "left_knee", "left_ankle", "left_foot", "right_hip", "right_knee", "right_ankle", "right_foot"};

xn::Context			g_Context;
xn::DepthGenerator	g_DepthGenerator;
xn::UserGenerator	g_UserGenerator;
xn::SceneAnalyzer	g_SceneAnalyzer;
xn::ImageGenerator	g_ImageGenerator;

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator&, XnUserID, void*);
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator&, XnUserID, void*);
void XN_CALLBACK_TYPE User_OutOfScene(xn::UserGenerator&, XnUserID, void*);
void XN_CALLBACK_TYPE User_BackIntoScene(xn::UserGenerator&, XnUserID, void*);

XnUInt32 calibrationData;

void publishUserTransforms(XnUserID const&, std::string const&);
void publishTransforms(const std::string&);
bool checkCenterOfMass(XnUserID const&);
void drawUsersPixels();
void drawUserPixels(XnUserID const&, cv::Mat&);
void updateRGBImage(const sensor_msgs::ImageConstPtr&);

std::vector<image_transport::Publisher> pubs(MAX_USERS);

cv::Mat RGBImage;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multiple_skeleton_tracker");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, updateRGBImage);

  std::string configFilename = ros::package::getPath("hostess_skeleton_tracker") + "/init/openni_tracker.xml";
  genericUserCalibrationFileName = ros::package::getPath("hostess_skeleton_tracker") + "/init/GenericUserCalibration.bin";

  XnStatus nRetVal;

  while(nh.ok())
  {
    nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());

    if(nRetVal != XN_STATUS_OK)
    {
      ROS_INFO("InitFromXml failed: %s Retrying in 3 seconds...", xnGetStatusString(nRetVal));
      ros::Duration(3).sleep();
    }
    else
    {
      break;
    }
  }

  std::string frame_id;

  nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);

  if(nRetVal != XN_STATUS_OK)
  {
    ROS_ERROR("Find depth generator failed: %s", xnGetStatusString(nRetVal));
  }

  frame_id = "camera_depth_frame";

  nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_ImageGenerator);

  if(nRetVal != XN_STATUS_OK)
  {
    nRetVal = g_ImageGenerator.Create(g_Context);
  }

  if(nRetVal != XN_STATUS_OK)
  {
    ROS_ERROR("Find image generator failed: %s", xnGetStatusString(nRetVal));
  }

  nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);

  if(nRetVal != XN_STATUS_OK)
  {
    nRetVal = g_UserGenerator.Create(g_Context);

    if (nRetVal != XN_STATUS_OK)
    {
      ROS_ERROR("NITE is likely missing: Please install NITE >= 1.5.2.21. Check the readme for download information. Error Info: User generator failed: %s", xnGetStatusString(nRetVal));
      return nRetVal;
    }
  }

  if(!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
  {
    ROS_INFO("Supplied user generator doesn't support skeleton");
    return EXIT_FAILURE;
  }

  g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

  XnCallbackHandle hUserCallbacks;
  g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
  g_UserGenerator.RegisterToUserExit(User_OutOfScene, NULL, hUserCallbacks);
  g_UserGenerator.RegisterToUserReEnter(User_BackIntoScene, NULL, hUserCallbacks);

  if(g_DepthGenerator.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT))
  {
    g_DepthGenerator.GetAlternativeViewPointCap().SetViewPoint(g_ImageGenerator);
    g_UserGenerator.GetAlternativeViewPointCap().SetViewPoint(g_ImageGenerator);
    frame_id = "camera_rgb_frame";
  }

  nh.setParam("camera_frame_id", frame_id);

  nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_SCENE, g_SceneAnalyzer);

  if(nRetVal != XN_STATUS_OK)
  {
    nRetVal = g_SceneAnalyzer.Create(g_Context);
  }

  if(nRetVal != XN_STATUS_OK)
  {
    ROS_ERROR("Find scene analyzer failed: %s", xnGetStatusString(nRetVal));
  }

  nRetVal = g_Context.StartGeneratingAll();

  if(nRetVal != XN_STATUS_OK)
  {
    ROS_ERROR("StartGenerating failed: %s", xnGetStatusString(nRetVal));
  }

  for(int i = 0; i < pubs.size(); i++)
  {
    std::ostringstream oss;

    oss << "users_blobs/user_" << i + 1;

    pubs[i] = it.advertise(oss.str(), 1);
  }

  while(nh.ok())
  {
    ros::spinOnce();
    g_Context.WaitAndUpdateAll();
    publishTransforms(frame_id);
    drawUsersPixels();
    ros::Rate(30).sleep();
  }

  g_Context.Shutdown();
  return EXIT_SUCCESS;
}

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
  ROS_INFO("New User %d.", nId);

  if(calibrationData == NULL)
  {
    g_UserGenerator.GetSkeletonCap().LoadCalibrationDataFromFile(nId, genericUserCalibrationFileName.c_str());
    g_UserGenerator.GetSkeletonCap().SaveCalibrationData(nId, calibrationData);
  }
  else
  {
    g_UserGenerator.GetSkeletonCap().LoadCalibrationData(nId, calibrationData);
  }

  g_UserGenerator.GetSkeletonCap().StartTracking(nId);
  ROS_INFO("Start tracking user: %d.", nId);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
  ROS_INFO("Lost user %d. Stop tracking.", nId);

  g_UserGenerator.GetSkeletonCap().StopTracking(nId);
}

void XN_CALLBACK_TYPE User_BackIntoScene(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
  ROS_INFO("User %d back into scene. Restart tracking.", nId);

  g_UserGenerator.GetSkeletonCap().StartTracking(nId);
}

void XN_CALLBACK_TYPE User_OutOfScene(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
  ROS_INFO("User %d out of scene. Stop tracking.", nId);

  g_UserGenerator.GetSkeletonCap().StopTracking(nId);
}

void publishUserTransforms(XnUserID const& user, std::string const& frame_id)
{
  static tf::TransformBroadcaster broadcaster;

  XnSkeletonJointPosition joint_position[24];
  XnSkeletonJointOrientation joint_orientation[24];

  if(g_UserGenerator.GetSkeletonCap().IsTracking(user))
  {
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_HEAD, joint_position[0]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_HEAD, joint_orientation[0]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_NECK, joint_position[1]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_NECK, joint_orientation[1]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_TORSO, joint_position[2]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_TORSO, joint_orientation[2]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_WAIST, joint_position[3]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_WAIST, joint_orientation[3]);

    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_COLLAR, joint_position[4]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_LEFT_COLLAR, joint_orientation[4]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_SHOULDER, joint_position[5]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_LEFT_SHOULDER, joint_orientation[5]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_ELBOW, joint_position[6]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_LEFT_ELBOW, joint_orientation[6]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_WRIST, joint_position[7]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_LEFT_WRIST, joint_orientation[7]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_HAND, joint_position[8]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_LEFT_HAND, joint_orientation[8]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_FINGERTIP, joint_position[9]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_LEFT_FINGERTIP, joint_orientation[9]);

    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_COLLAR, joint_position[10]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_RIGHT_COLLAR, joint_orientation[10]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_SHOULDER, joint_position[11]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_RIGHT_SHOULDER, joint_orientation[11]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_ELBOW, joint_position[12]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_RIGHT_ELBOW, joint_orientation[12]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_WRIST, joint_position[13]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_RIGHT_WRIST, joint_orientation[13]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_HAND, joint_position[14]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_RIGHT_HAND, joint_orientation[14]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_FINGERTIP, joint_position[15]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_RIGHT_FINGERTIP, joint_orientation[15]);

    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_HIP, joint_position[16]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_LEFT_HIP, joint_orientation[16]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_KNEE, joint_position[17]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_LEFT_KNEE, joint_orientation[17]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_ANKLE, joint_position[18]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_LEFT_ANKLE, joint_orientation[18]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_FOOT, joint_position[19]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_LEFT_FOOT, joint_orientation[19]);

    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_HIP, joint_position[20]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_RIGHT_HIP, joint_orientation[20]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_KNEE, joint_position[21]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_RIGHT_KNEE, joint_orientation[21]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_ANKLE, joint_position[22]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_RIGHT_ANKLE, joint_orientation[22]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_FOOT, joint_position[23]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_RIGHT_FOOT, joint_orientation[23]);
  }
  else
  {
    return;
  }

  ros::Time now = ros::Time::now();

  static tf::TransformListener listener;

  tf::StampedTransform parentTransform;

  bool sendGlobal;

  try
  {
    listener.lookupTransform("map", frame_id, ros::Time(0), parentTransform);

    sendGlobal = true;
  }
  catch(tf::TransformException& ex)
  {
    sendGlobal = false;
  }

  for(int i = 0; i < 24; i++)
  {
    double x = -joint_position[i].position.X / 1000.0;
    double y = joint_position[i].position.Y / 1000.0;
    double z = joint_position[i].position.Z / 1000.0;

    XnFloat* m = joint_orientation[i].orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);

    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    std::ostringstream oss, global_oss;

    oss << "user_" << user << "_" << joint_names[i];
    global_oss << "user_" << user << "_global_" << joint_names[i];

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));

    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;

    if(transform.getOrigin().getX() == transform.getOrigin().getX() && transform.getOrigin().getY() == transform.getOrigin().getY() && transform.getOrigin().getZ() == transform.getOrigin().getZ() && transform.getRotation().getX() == transform.getRotation().getX() && transform.getRotation().getY() == transform.getRotation().getY() && transform.getRotation().getZ() == transform.getRotation().getZ() && transform.getRotation().getW() == transform.getRotation().getW())
    {
      broadcaster.sendTransform(tf::StampedTransform(transform, now, frame_id, oss.str()));
    }

    if(sendGlobal)
    {
      tf::Transform global = parentTransform * transform;

      if(global.getOrigin().getX() == global.getOrigin().getX() && global.getOrigin().getY() == global.getOrigin().getY() && global.getOrigin().getZ() == global.getOrigin().getZ() && global.getRotation().getX() == global.getRotation().getX() && global.getRotation().getY() == global.getRotation().getY() && global.getRotation().getZ() == global.getRotation().getZ() && global.getRotation().getW() == global.getRotation().getW())
      {
        broadcaster.sendTransform(tf::StampedTransform(global, now, "map", global_oss.str()));
      }
    }
  }
}

void publishTransforms(const std::string& frame_id)
{
  ros::Time now = ros::Time::now();

  XnUInt16 users_count = MAX_USERS;
  XnUserID users[MAX_USERS];

  g_UserGenerator.GetUsers(users, users_count);

  for(int i = 0; i < users_count; ++i)
  {
    XnUserID user = users[i];

    if(checkCenterOfMass(user))
    {
      publishUserTransforms(user, frame_id);
    }
  }
}

void updateRGBImage(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat image;

  try
  {
    image = cv_bridge::toCvShare(msg, "bgr8")->image;

    if(image.empty())
    {
      return;
    }

    RGBImage = image.clone();
  }
  catch(cv_bridge::Exception& e)
  {
    //Conversion failed
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());

    return;
  }
}

void drawUsersPixels()
{
  XnUInt16 users_count = MAX_USERS;
  XnUserID users[MAX_USERS];

  g_UserGenerator.GetUsers(users, users_count);

  for(int i = 0; i < users_count; ++i)
  {
    XnUserID user = users[i];

    if(checkCenterOfMass(user))
    {
      cv::Mat maskedImage;

      drawUserPixels(user, maskedImage);

      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", maskedImage).toImageMsg();

      pubs[user - 1].publish(msg);
    }
  }
}

void drawUserPixels(XnUserID const& user, cv::Mat& maskedImage)
{
  xn::SceneMetaData smd;
  xn::DepthMetaData dmd;

  g_DepthGenerator.GetMetaData(dmd);
  g_SceneAnalyzer.GetMetaData(smd);

  const XnLabel* pLabels = smd.Data();

  cv::Mat maskImage = cv::Mat(dmd.FullYRes(), dmd.FullXRes(), CV_8UC1);

  for(XnUInt y = 0; y < dmd.YRes(); ++y)
  {
    for(XnUInt x = 0; x < dmd.XRes(); ++x, ++pLabels)
    {
      maskImage.at<uchar>(y, x) = (*pLabels == user) ? 255 : 0;
    }
  }

  RGBImage.copyTo(maskedImage, maskImage);
}

bool checkCenterOfMass(XnUserID const& user)
{
  XnPoint3D center_of_mass;
  XnStatus status = g_UserGenerator.GetCoM(user, center_of_mass);

  if(status != XN_STATUS_OK || (center_of_mass.X == 0 && center_of_mass.Y == 0 && center_of_mass.Z == 0))
  {
    return false;
  }
  else
  {
    return true;
  }
}
