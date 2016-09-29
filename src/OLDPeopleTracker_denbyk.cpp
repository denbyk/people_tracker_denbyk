#include <ros/ros.h>
#include <ros/package.h>
#include <string>

#include <ni/XnCppWrapper.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

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
    publishTransforms(frame_id);
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

/*publishing-transforms---------------------------------------------------------------------------------------------*/
void publishTrackings()



void publishTransforms(const std::string& frame_id)
{
  ros::Time now = ros::Time::now();

  XnUInt16 users_count = MAX_USERS;
  XnUserID users[MAX_USERS];

  g_UserGenerator.GetUsers(users, users_count);

  //per ogni utente pubblica la trasformazione
  for(int i = 0; i < users_count; ++i)
  {
    XnUserID user = users[i];

    if(checkCenterOfMass(user))
    {
      publishUserTransforms(user, frame_id);
    }
  }
}

//calcola e pubblica posizione e velocità di user
void publishUserTracking(XnUserID const& user, ros::NodeHandler nh)
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
  //TODO: nota: ci deve essere un unico topic per tutti gli utenti, è sbagliato farlo qui. va fatto nella funzione che li pubblica tutti
  track_pub = nh.advertise("user_tracking", 1000);

}



/*-----------------------------------------------------------------*/
void publishUserTransforms(XnUserID const& user, std::string const& frame_id)
{
  static tf::TransformBroadcaster broadcaster;

  //ottiene posizione e orientamento dal tracker di torso e testa dell'utente user
  XnSkeletonJointPosition joint_position[2];
  XnSkeletonJointOrientation joint_orientation[2];

  if(g_UserGenerator.GetSkeletonCap().IsTracking(user))
  {
    //ottiene posizione e orientamento di testa e torso
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_TORSO, joint_position[0]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_TORSO, joint_orientation[0]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_HEAD, joint_position[1]);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_HEAD, joint_orientation[1]);

    if(joint_position[0].fConfidence < 1 || joint_position[1].fConfidence < 1)
    {
      return;
    }
  }
  else
  {
    return;
  }

  ros::Time now = ros::Time::now();

  //listener per ascolto di posizione frame camera rispetto al mondo
  static tf::TransformListener listener;
  tf::StampedTransform parentTransform;

  bool sendGlobal;

  try
  {
    //ascolta la trasformazione da frame_id (depth_camera_frame) a map, prendendo il frame + recente e memorizza in parent transform
    listener.lookupTransform("map", frame_id, ros::Time(0), parentTransform);
    sendGlobal = true;
  }
  catch(tf::TransformException& ex)
  {
    sendGlobal = false;
  }

  for(int i = 0; i < 2; i++)
  {
    //sia per testa che per torso, estrai x,y,z (posizioni)
    double x = -joint_position[i].position.X / 1000.0;
    double y = joint_position[i].position.Y / 1000.0;
    double z = joint_position[i].position.Z / 1000.0;

    //estrae matrice di rotazione
    XnFloat* m = joint_orientation[i].orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);

    double qx, qy, qz, qw;
    //è passato da KDL per estrarre il quaternione
    rotation.GetQuaternion(qx, qy, qz, qw);

    std::ostringstream oss, global_oss;

    //output
    if(i == 0)
    {
      oss << "torso_" << user;
      global_oss << "global_torso_" << user;
    }
    else
    {
      oss << "head_" << user;
      global_oss << "global_head_" << user;
    }

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));

    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(1.5708, 0, 1.5708); //<????????????????????????????????????? forse per bloccare la camera orientata in un certo modo.
    change_frame.setRotation(frame_rotation);

    //transform = trasformazione ogg finale
    //change_frame serve per modificare la trasformazione in base a posizionamento e inclinazione telecamera
    transform = change_frame * transform;

    //pubblica la trasformazione
    broadcaster.sendTransform(tf::StampedTransform(transform, now, frame_id, oss.str()));

    if(sendGlobal)
    {
      //pubblica trasformazinoe globale
      tf::Transform global = parentTransform * transform;
      broadcaster.sendTransform(tf::StampedTransform(global, now, "map", global_oss.str()));
    }
  }
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
