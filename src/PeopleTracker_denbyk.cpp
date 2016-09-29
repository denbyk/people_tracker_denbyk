#include <ros/ros.h>
#include <ros/package.h>
#include <string>

#include <ni/XnCppWrapper.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <people_tracker_denbyk_msg/UserTracking.h>

#include <kdl/frames.hpp>
/*
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
*/

#define MAX_USERS 6

//contesto openNI
xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

//dichiarazioni funzioni
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE User_BackIntoScene(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE User_OutOfScene(xn::UserGenerator& generator, XnUserID nId, void* pCookie);

XnUInt32 calibrationData;
std::string genericUserCalibrationFileName;

void publishTransforms(const std::string& frame_id);
void publishUserTransforms(XnUserID const& user, std::string const& frame_id);
bool checkCenterOfMass(XnUserID const& user);


ros::Publisher track_pub;

int main(int argc, char **argv){
  ros::init(argc, argv, "pplTracker");
  ros::NodeHandle nh;

  std::string configFilename = ros::package::getPath("people_tracker_denbyk") + "/init/openni_tracker.xml";
  genericUserCalibrationFileName = ros::package::getPath("people_tracker_denbyk") + "/init/GenericUserCalibration.bin";

  ros::Rate loop_rate(1);

  //valore di ritorno Xn
  XnStatus nRetVal;

  //while (ros::ok())
  while (nh.ok())
  {
    //inizializzo contesto openni
    //ROS_INFO(configFilename.c_str(),xnGetStatusString(nRetVal));
    nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());

    //TODO: remove
    nRetVal = XN_STATUS_OK;

    //riprovo tra un po'
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


  //std::string frame_id;

  nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);


  if(nRetVal != XN_STATUS_OK)
  {
    ROS_ERROR("Find depth generator failed: %s", xnGetStatusString(nRetVal));
  }

  //cerca nodo ti tipo user generator e lo salva in g_UserGenerator
  nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);


  if (nRetVal != XN_STATUS_OK)
  {
    //crea lo userGenerator del g_context. SE non riesce probabilmente manca NITE
    nRetVal = g_UserGenerator.Create(g_Context);

    if (nRetVal != XN_STATUS_OK)
    {
      ROS_ERROR("NITE is likely missing: Please install NITE >= 1.5.2.21. Check the readme for download information. Error Info: User generator failed: %s", xnGetStatusString(nRetVal));
      return nRetVal;
    }
  }
  //veriica che lo userGenerator creato supporti SKELETON
  if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
  {
    ROS_INFO("Supplied user generator doesn't support skeleton");
    return EXIT_FAILURE;
  }

  //imposto la modalità dello skeleton, quali giunzioni rilevare e quali no.
  //in questo caso upper è il torso/la testa
  g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_UPPER);

  //setto varie callbacks per entrata, uscita e rientro user nello UserGenerator
  XnCallbackHandle hUserCallbacks;
  g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
  g_UserGenerator.RegisterToUserExit(User_OutOfScene, NULL, hUserCallbacks);
  g_UserGenerator.RegisterToUserReEnter(User_BackIntoScene, NULL, hUserCallbacks);

  //attivo la generazione dei vari generators
  nRetVal = g_Context.StartGeneratingAll();

  if(nRetVal != XN_STATUS_OK)
  {
    ROS_ERROR("StartGenerating failed: %s", xnGetStatusString(nRetVal));
  }

  //recupera un parametro passato al nodo dal server dei parametri ROS.
  //usando la chiave camera_frame_id e lo memorizza nella variabile frame_id
  std::string frame_id("camera_depth_frame");
  nh.getParam("camera_frame_id", frame_id);


  std::cout << "init ok\n";
  //ciclo principale.
  while(nh.ok())
  {
    //aggiorna il contesto e aspetta
    g_Context.WaitAndUpdateAll();
    //pubblica le trasformazioni su frame_id
    //publishTransforms(frame_id);
    //dormi per un tot.
    ros::Rate(30).sleep();
  }

  //rilascia risorse e esci.
  g_Context.Shutdown();
  return EXIT_SUCCESS;
}

/*callbacks---------------------------------------------------------------------------------------------*/

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
  ROS_INFO("New User %d.", nId);

  //se non ha ancora fatto calibrazione la carica da file e ?la salva su calibrationData?
  if(calibrationData == NULL)
  {
    g_UserGenerator.GetSkeletonCap().LoadCalibrationDataFromFile(nId, genericUserCalibrationFileName.c_str());
    g_UserGenerator.GetSkeletonCap().SaveCalibrationData(nId, calibrationData);
  }
  else //se no carica la calibration data???
  {
    g_UserGenerator.GetSkeletonCap().LoadCalibrationData(nId, calibrationData);
  }
  //inizia il tracking
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

/*publishing----------------------------------------------------------------------------------------------*/

void publishTrackings()
{
  track_pub = nh.advertise("people_tracking", 1000);
}

//calcola e pubblica posizione e velocità di user
void publishSingleUserTracking(XnUserID const& user, ros::NodeHandler nh)
{
  //ottiene posizione e orientamento busto di user
  XnSkeletonJointPosition joint_position;
  XnSkeletonJointOrientation joint_orientation;

  if(g_UserGenerator.GetSkeletonCap().IsTracking(user))
  {
    //ottiene posizione e orientamento di testa e torso
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_TORSO, joint_position);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_TORSO, joint_orientation);

    if(joint_position.fConfidence < 1 || joint_position.fConfidence < 1)
    {
      return;
    }
  }
  else
  {
    return;
  }
  //estrai x,y,z (posizioni)
  double x = -joint_position[i].position.X / 1000.0;
  double y = joint_position[i].position.Y / 1000.0;
  double z = joint_position[i].position.Z / 1000.0;

  //TODO: ma la rotazione serve?
  //estrae matrice di rotazione
  XnFloat* m = joint_orientation[i].orientation.elements;
  KDL::Rotation rotation(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
  double qx, qy, qz, qw;
  rotation.GetQuaternion(qx, qy, qz, qw);

  //output
  oss << "torso_" << user;

  //calcola velocità tra istanti successivi
  //pubblica dati
  
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
