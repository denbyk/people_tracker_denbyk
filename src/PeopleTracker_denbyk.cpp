#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <math.h>

#include <ni/XnCppWrapper.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <kdl/frames.hpp>

#include <people_tracker_denbyk_msg/UserTracking.h>
#include <sensor_msgs/Image.h>
#include <opencv2/video/tracking.hpp>
/*
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
*/

/*TODO memo:
   no kalman
   filtrare velocità piccole
   idea: filtro passa basso?
   orientamento torso da fermo (openni lo fa?)
   usare msg di ros people ?

*/

//settings
#define NOCAMERA_MODE false
#define WHAT_TO_TRACK XN_SKEL_TORSO //XN_SKEL_HEAD
#define MAX_USERS 2
#define min_position_precision 0.001
#define min_velocity_precision 0.03


typedef people_tracker_denbyk_msg::SingleUserTracking SingleUserTracking;
typedef people_tracker_denbyk_msg::UserTracking UserTracking;

//contesto openNI
xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

//nocamera_mode
xn::MockDepthGenerator g_mock_depth;
sensor_msgs::Image currDepthImage;

//dichiarazioni funzioni
void openni_init(ros::NodeHandle& nh);
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE User_BackIntoScene(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE User_OutOfScene(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
void updateDepthImage(const sensor_msgs::Image& dephImageMsg);
double FloorTo(double num, double LeastSignificantChange);

XnUInt32 calibrationData;
std::string genericUserCalibrationFileName;

void publishTrackings(ros::Publisher& track_pub);
SingleUserTracking calcSingleUserTracking(XnUserID const& user, SingleUserTracking& PrevTracking);
bool checkCenterOfMass(XnUserID const& user);


std::map<XnUserID,SingleUserTracking> PrevTrackings;
ros::Publisher track_pub;
SingleUserTracking NullUserTracking;



int main(int argc, char **argv){
   ros::init(argc, argv, "pplTracker");
   ros::NodeHandle nh;

   openni_init(nh);

   PrevTrackings;// = new std::map<XnUserID,SingleUserTracking>();
   NullUserTracking.UserID = -1;

   //init msg
   track_pub = nh.advertise<UserTracking>("people_tracking", 100);
   std::cout << "init ok\n";
   //DEBUGtrack_pub = nh.advertise<SingleUserTracking>("DEBUG_people_tracking", 100);

   //ciclo principale.
   while(nh.ok())
   {
      if (NOCAMERA_MODE)
      {
         // Update data
         g_Context.WaitOneUpdateAll(g_DepthGenerator);
         // Get original depth map
         xn::DepthMetaData depthMD;
         g_DepthGenerator.GetMetaData(depthMD);
         // Make data writable and modify
         depthMD.MakeDataWritable();

         // Modify data ....
         //write a function which changes the depth data in your original depth map
         xn::DepthMap& depthMap = depthMD.WritableDepthMap();
         for(unsigned int y = 0; y < depthMap.YRes(); ++ y)
            for(unsigned int x = 0; x < depthMap.XRes(); ++ x)
               depthMap (x, y) = 0; // replace the values of the depth map by recorded depth map
         //depthMap(x,y) = subscribed_depth< data_type >.at(x,y);
         // set the data of the mock-depth generator and pose will tracking will run on this
         g_mock_depth.SetData(depthMD);
      }
      else
      {
         //std::cout << "nh okok\n";
         //aggiorna il contesto e aspetta
         //std::cout<<"pre\n";
         g_Context.WaitAndUpdateAll();
         //std::cout<<"post\n";
      }
      //pubblica le trasformazioni su frame_id
      publishTrackings(track_pub);
      //dormi per un tot.
      ros::Rate(30).sleep();
   }

   //rilascia risorse e esci.
   g_Context.Shutdown();
   return EXIT_SUCCESS;
}


/*publishing----------------------------------------------------------------------------------------------*/

void publishTrackings(ros::Publisher& track_pub)
{
   XnUInt16 users_count = MAX_USERS;
   XnUserID users[MAX_USERS];
   UserTracking allTrackings;

   g_UserGenerator.GetUsers(users, users_count);

   //per ogni utente pubblica il tracking
   for(int i = 0; i < users_count; i++)
   {
      XnUserID CurrUser = users[i];
      SingleUserTracking thisUserPrevTracking;
      try
      {
         thisUserPrevTracking = PrevTrackings.at(CurrUser);
      }
      catch(std::out_of_range&)
      {
         thisUserPrevTracking = NullUserTracking;
      }
      SingleUserTracking sut = calcSingleUserTracking(CurrUser, thisUserPrevTracking);

      if (sut.UserID == -1)
      {
         std::cout << "user[" << i << "] è null. userCount = " << users_count << "\n";
         continue;
      }

      allTrackings.user.push_back(sut);
      PrevTrackings[CurrUser] = sut;
      //DEBUGtrack_pub.publish(allTrackings.user[i]);
   }
   allTrackings.userCount = users_count;

   track_pub.publish(allTrackings);
}

//calcola e pubblica posizione e velocità di user

SingleUserTracking calcSingleUserTracking(XnUserID const& user, SingleUserTracking& PrevTracking)
{
   //std::cout<<"calcSingleUserTracking\n";

   //ottiene posizione e orientamento busto di user
   XnSkeletonJointPosition joint_position;
   XnSkeletonJointOrientation joint_orientation;


   if(g_UserGenerator.GetSkeletonCap().IsTracking(user))
   {
      //ottiene posizione e orientamento di testa e torso
      g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, WHAT_TO_TRACK, joint_position);
      g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, WHAT_TO_TRACK, joint_orientation);


      if(joint_position.fConfidence < 1)
      {
         //std::cout << "no confidence f: " << joint_position.fConfidence << "\n";
         return NullUserTracking;
      }
   }
   else
   {
      //std::cout << "no tracking\n";
      return NullUserTracking;
   }
   //estrai x,y,z (posizioni)
   double x = -joint_position.position.X/1000;
   double y = joint_position.position.Y/1000;
   double z = joint_position.position.Z/1000;

   //floor to mm
   x = FloorTo(x, min_position_precision);
   y = FloorTo(y, min_position_precision);
   z = FloorTo(z, min_position_precision);

   //TODO: ma la rotazione serve?
   //estrae matrice di rotazione
   XnFloat* m = joint_orientation.orientation.elements;
   KDL::Rotation rotation(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
   double qx, qy, qz, qw;
   rotation.GetQuaternion(qx, qy, qz, qw);
   std::cout << m[0] << "\t" << m[1] << "\t" << m[2] << "\n";
   std::cout << m[3] << "\t" << m[4] << "\t" << m[5] << "\n";
   std::cout << m[6] << "\t" << m[7] << "\t" << m[8] << "\n";
   std::cout << "----------------------\n";
   //output
   //oss << "torso_" << user;

   SingleUserTracking sut;

   sut.timestamp = ros::Time::now();
   sut.UserID = user;
   sut.Pos_X = x;
   sut.Pos_Y = y;
   sut.Pos_Z = z;

   if (PrevTracking.UserID == -1)//PrevTracking == NullUserTracking
   {
      //è il primo tracking per questo utente, nn ho info su stato precedente. parte "da fermo"
      sut.Vel_X = 0;
      sut.Vel_Y = 0;
      sut.Vel_Z = 0;
   }
   else
   {
      ros::Duration dur = sut.timestamp - PrevTracking.timestamp;
      double time_delay = dur.toSec();
      //calcola velocità tra istanti successivi
      double Vel_X = (x - PrevTracking.Pos_X) / time_delay;
      double Vel_Y = (y - PrevTracking.Pos_Y) / time_delay;
      double Vel_Z = (z - PrevTracking.Pos_Z) / time_delay;

      sut.Vel_X = FloorTo(Vel_X, min_velocity_precision);
      sut.Vel_Y = FloorTo(Vel_Y, min_velocity_precision);
      sut.Vel_Z = FloorTo(Vel_Z, min_velocity_precision);
   }
   return sut;
}

double FloorTo(double num, double LeastSignificantChange)
{
   return floor(num/LeastSignificantChange)*LeastSignificantChange;
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

/*callbacks---------------------------------------------------------------------------------------------*/

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
   std::cout << "newUser\n";
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

   PrevTrackings.erase(nId);
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

   PrevTrackings.erase(nId);
}

/*-------------------Openni_init----------------------------------------------*/
void openni_init(ros::NodeHandle& nh)
{
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

   if(NOCAMERA_MODE)
   {
      ros::Subscriber sub = nh.subscribe("/camera/depth/image_raw", 1000, updateDepthImage);
      // Depth generator
      g_DepthGenerator.Create(g_Context);
      // Mock depth generator
      g_mock_depth.CreateBasedOn(g_DepthGenerator,"the mock-depth");
      // Create user generator using mock depth generator
      xn::Query g_Query;
      g_Query.AddNeededNode("mock-depth");
      g_UserGenerator.Create(g_Context, &g_Query);
   }
   else
   {
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
         //crea lo userGenerator del g_Context. SE non riesce probabilmente manca NITE
         nRetVal = g_UserGenerator.Create(g_Context);
         if (nRetVal != XN_STATUS_OK)
         {
            ROS_ERROR("NITE is likely missing: Please install NITE >= 1.5.2.21. Check the readme for download information. Error Info: User generator failed: %s", xnGetStatusString(nRetVal));
            exit(nRetVal);
         }
      }
   }

   //veriica che lo userGenerator creato supporti SKELETON
   if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
   {
      ROS_INFO("Supplied user generator doesn't support skeleton");
      exit(EXIT_FAILURE);
   }
   //imposto la modalità dello skeleton, quali giunzioni rilevare e quali no.
   //in questo caso upper è il torso/la testa
   g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_UPPER);
   //setto varie callbacks per entrata, uscita e rientro user nello UserGenerator
   XnCallbackHandle hUserCallbacks;
   g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
   g_UserGenerator.RegisterToUserExit(User_OutOfScene, NULL, hUserCallbacks);
   g_UserGenerator.RegisterToUserReEnter(User_BackIntoScene, NULL, hUserCallbacks); //< ma viene mai usato????
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
}

void updateDepthImage(const sensor_msgs::Image& depthImageMsg)
{
   currDepthImage = depthImageMsg;
   std::cout << "updateDepthImage";
}
