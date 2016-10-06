#include <ros/ros.h>
#include <ros/package.h>
#include <string>

#include <ni/XnCppWrapper.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <kdl/frames.hpp>

#include <people_tracker_denbyk_msg/UserTracking.h>

#include <opencv2/video/tracking.hpp>


//----------------kalman filter-----------
const int stateSize = 6;
const int measSize = 3;
const int contrSize = 0;

cv::KalmanFilter kf1(stateSize, measSize, contrSize);
cv::Mat state(stateSize, 1, CV_32F);
cv::Mat meas(measSize, 1, CV_32F);
//cv::Point oldPosition;

//----------------------------------------

#define MAX_USERS 1

typedef people_tracker_denbyk_msg::SingleUserTracking SingleUserTracking;
typedef people_tracker_denbyk_msg::UserTracking UserTracking;

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

void publishTrackings(ros::Publisher& track_pub);
SingleUserTracking calcSingleUserTracking(XnUserID const& user, SingleUserTracking& PrevTracking);
bool checkCenterOfMass(XnUserID const& user);


ros::Publisher track_pub;
ros::Publisher DEBUGtrack_pub;

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

  //kalman filter
  kalmanInitialization();

  std::cout << "init ok\n";

  //init msg
  track_pub = nh.advertise<UserTracking>("people_tracking", 100);
  DEBUGtrack_pub = nh.advertise<SingleUserTracking>("DEBUG_people_tracking", 100);
  //ciclo principale.
  while(nh.ok())
  {
    //std::cout << "nh okok\n";
    //aggiorna il contesto e aspetta
    //std::cout<<"pre\n";
    g_Context.WaitAndUpdateAll();
    //std::cout<<"post\n";
    //pubblica le trasformazioni su frame_id
    publishTrackings(track_pub);
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

void publishTrackings(ros::Publisher& track_pub)
{
  XnUInt16 users_count = MAX_USERS;
  XnUserID users[MAX_USERS];
  UserTracking allTrackings;

  g_UserGenerator.GetUsers(users, users_count);

  SingleUserTracking PrevTracking[users_count];

  //per ogni utente pubblica il tracking
  for(int i = 0; i < users_count; i++)
  {
    XnUserID CurrUser = users[i];

    SingleUserTracking sut = calcSingleUserTracking(CurrUser, PrevTracking[i]);

    if (sut.UserID == -1)
    {
      std::cout << "user[" << i << "] è null. userCount = " << users_count << "\n";
    }

    allTrackings.user.push_back(sut);
    PrevTracking[i] = sut;
    DEBUGtrack_pub.publish(allTrackings.user[i]);
  }
  allTrackings.userCount = users_count;

  track_pub.publish(allTrackings);
}

//calcola e pubblica posizione e velocità di user

SingleUserTracking calcSingleUserTracking(XnUserID const& user, SingleUserTracking& PrevTracking)
{
  //std::cout<<"calcSingleUserTracking\n";

  SingleUserTracking NullUserTracking;
  NullUserTracking.UserID = -1;

  //ottiene posizione e orientamento busto di user
  XnSkeletonJointPosition joint_position;
  XnSkeletonJointOrientation joint_orientation;


  if(g_UserGenerator.GetSkeletonCap().IsTracking(user))
  {
    //ottiene posizione e orientamento di testa e torso
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_TORSO, joint_position);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_TORSO, joint_orientation);


    if(joint_position.fConfidence < 1/* || joint_position.fConfidence < 1*/)
    {
      std::cout << "no confidence f: " << joint_position.fConfidence << "\n";
      return NullUserTracking;
    }
  }
  else
  {
    std::cout << "no tracking\n";
    return NullUserTracking;
  }
  //estrai x,y,z (posizioni)
  double x = -joint_position.position.X / 1000.0;
  double y = joint_position.position.Y / 1000.0;
  double z = joint_position.position.Z / 1000.0;

  //TODO: ma la rotazione serve?
  //estrae matrice di rotazione
  // XnFloat* m = joint_orientation.orientation.elements;
  // KDL::Rotation rotation(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
  // double qx, qy, qz, qw;
  // rotation.GetQuaternion(qx, qy, qz, qw);

  //output
  //oss << "torso_" << user;

  SingleUserTracking sut;

  sut.timestamp = ros::Time::now();
  sut.UserID = user;
  sut.Pos_X = x;
  sut.Pos_Y = y;
  sut.Pos_Z = z;

  double time_delay = (sut.timestamp.toNSec() - PrevTracking.timestamp.toNSec()) * 1000;

  //calcola velocità tra istanti successivi
  sut.Vel_X = (x - PrevTracking.Pos_X) / time_delay;
  sut.Vel_Y = (y - PrevTracking.Pos_Y) / time_delay;
  sut.Vel_Z = (z - PrevTracking.Pos_Z) / time_delay;

  return sut;
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


//------------------------- Kalman Filter methods --------------------------

void kalmanInitialization()
{
	cv::setIdentity(kf1.transitionMatrix);
	kf1.transitionMatrix.at<float>(3) = 1.0f;
	kf1.transitionMatrix.at<float>(10) = 1.0f;
	kf1.transitionMatrix.at<float>(17) = 1.0f;

	kf1.measurementMatrix = cv::Mat::zeros(measSize, stateSize, CV_32F);
	kf1.measurementMatrix.at<float>(0) = 1.0f;
	kf1.measurementMatrix.at<float>(7) = 1.0f;
	kf1.measurementMatrix.at<float>(14) = 1.0f;

	cv::setIdentity(kf1.processNoiseCov, cv::Scalar(1e-2));
	kf1.processNoiseCov.at<float>(21) = 1e-1;
	kf1.processNoiseCov.at<float>(28) = 1e-1;
	kf1.processNoiseCov.at<float>(35) = 1e-1;

	cv::setIdentity(kf1.measurementNoiseCov, cv::Scalar(1e-1));
	cv::setIdentity(kf1.errorCovPost, cv::Scalar(1e-1));

	// cv::setIdentity(kf2.transitionMatrix);
	// kf2.transitionMatrix.at<float>(3) = 1.0f;
	// kf2.transitionMatrix.at<float>(10) = 1.0f;
	// kf2.transitionMatrix.at<float>(17) = 1.0f;
  //
	// kf2.measurementMatrix = cv::Mat::zeros(measSize, stateSize, CV_32F);
	// kf2.measurementMatrix.at<float>(0) = 1.0f;
	// kf2.measurementMatrix.at<float>(7) = 1.0f;
	// kf2.measurementMatrix.at<float>(14) = 1.0f;
  //
	// cv::setIdentity(kf2.processNoiseCov, cv::Scalar(1e-2));
	// kf2.processNoiseCov.at<float>(21) = 1e-1;
	// kf2.processNoiseCov.at<float>(28) = 1e-1;
	// kf2.processNoiseCov.at<float>(35) = 1e-1;
  //
	// cv::setIdentity(kf2.measurementNoiseCov, cv::Scalar(1e-1));
	// cv::setIdentity(kf2.errorCovPost, cv::Scalar(1e-1));
}

void predictAndPublish()
{
	kalmanPrediction();

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(state.at<float>(0), state.at<float>(1), state.at<float>(2)));
	transform.setRotation(tf::Quaternion(0, 0, 0, 1));

	static tf::TransformBroadcaster br;
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_id, "torso_k"));
}

void kalmanPrediction()
{
	kf2.transitionMatrix.at<float>(3) = dT;
	kf2.transitionMatrix.at<float>(10) = dT;
	kf2.transitionMatrix.at<float>(17) = dT;

	state = kf2.predict();
}

void kalmanUpdate(tf::Transform transform)
{
	if(!detected)
	{
		detected = true;

		kf1.statePre.at<float>(0) = transform.getOrigin().getX();
		kf1.statePre.at<float>(1) = transform.getOrigin().getY();
		kf1.statePre.at<float>(2) = transform.getOrigin().getZ();
		kf1.statePre.at<float>(3) = 0;
		kf1.statePre.at<float>(4) = 0;
		kf1.statePre.at<float>(5) = 0;

		kf2.statePre.at<float>(0) = transform.getOrigin().getX();
		kf2.statePre.at<float>(1) = transform.getOrigin().getY();
		kf2.statePre.at<float>(2) = transform.getOrigin().getZ();
		kf2.statePre.at<float>(3) = 0;
		kf2.statePre.at<float>(4) = 0;
		kf2.statePre.at<float>(5) = 0;

		kf1.predict();
		state = kf2.predict();

		oldPosition = cv::Point(state.at<float>(0), state.at<float>(1));
	}
	else
	{
		meas.at<float>(0) = transform.getOrigin().getX();
		meas.at<float>(1) = transform.getOrigin().getY();
		meas.at<float>(2) = transform.getOrigin().getZ();

		kf1.correct(meas);

		kf1.transitionMatrix.at<float>(3) = dT;
		kf1.transitionMatrix.at<float>(10) = dT;
		kf1.transitionMatrix.at<float>(17) = dT;

		state = kf1.predict();

		cv::Mat meas2(measSize, 1, CV_32F);

		meas2.at<float>(0) = state.at<float>(0);
		meas2.at<float>(1) = state.at<float>(1);
		meas2.at<float>(2) = state.at<float>(2);

		kf2.correct(meas2);
	}
}
