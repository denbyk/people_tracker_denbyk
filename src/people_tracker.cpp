#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <math.h>

#include <ni/XnCppWrapper.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <kdl/frames.hpp>

#include <people_tracker_msg/UserTracking.h>
#include <sensor_msgs/Image.h>
#include <opencv2/video/tracking.hpp>
#include <cv_bridge/cv_bridge.h>

#include <TrackingDenoiser.h>

//TODO:
/*
 * mettere rispetto a riferimento camera_link
 * pubblicare topic pose/point da vedere su rviz
 * volendo sistemare output video
 * aggiungere header
 */

//settings
#define WHAT_TO_TRACK XN_SKEL_TORSO //XN_SKEL_HEAD
#define MAX_USERS 5
#define min_position_precision 0.01
#define min_velocity_precision 0.01
#define VELOCITY_THRESHOLD 0.03
#define pi 3.14159265359
#define MOBILE_AVG_WINDOW_SIZE 10

typedef people_tracker_msg::SingleUserTracking SingleUserTracking;
typedef people_tracker_msg::UserTracking UserTracking;

//contesto openNI
xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;
xn::ImageGenerator g_ImageGenerator;

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
double ApplyNewPrecision(double num, double LeastSignificantChange);
void updateRgbImage(const sensor_msgs::Image& rgbImageMsg);
void VisualizeTracking(cv_bridge::CvImage& CvbImage, people_tracker_msg::UserTracking trackings);
double applyThreshold(double num, double threshold);
TrackedPoint CameraLinkToCameraDepthOpticalFrame(TrackedPoint old);
TrackedPoint CameraDepthOpticalFrameToCameraLink(TrackedPoint old);

XnUInt32 calibrationData;
std::string genericUserCalibrationFileName;

void publishTrackings(ros::Publisher& track_pub);
SingleUserTracking calcSingleUserTracking(XnUserID const& user, SingleUserTracking& PrevTracking, TrackingDenoiser& td);
//bool checkCenterOfMass(XnUserID const& user);


std::map<XnUserID,SingleUserTracking> PrevTrackings;
std::map<XnUserID,TrackingDenoiser> TrackingDenoisers;

ros::Publisher track_pub;
SingleUserTracking NullUserTracking;

//---tracking visualization---
bool VisualTrackingOn;
ros::Subscriber subImage;
cv_bridge::CvImage TrackingVisualizationMat;
sensor_msgs::Image TrackingVisualizationImage;
ros::Publisher TrackingVisualization_pub;


int main(int argc, char **argv){
	ros::init(argc, argv, "pplTracker");
	ros::NodeHandle nh;

	VisualTrackingOn = false;

	if(argc>2)
	{
		ROS_INFO("Errore nel numero di parametri. Lanciare il pacchetto senza parametri, con l'opzione -visualizeTracking o con -h per ulteriori informazioni.\n");
		return EXIT_FAILURE;
	}

	if (argc == 2 && std::string(argv[1]) == "-visualizeTracking")
	{
		VisualTrackingOn = true;
	}

	if (argc == 2 && std::string(argv[1]) == "-h")
	{
		ROS_INFO("Lanciare il pacchetto senza parametri o con l'opzione -visualizeTracking per ottenere il topic peopleTrackerVisualization da aprire con rviz\n");
	}

	openni_init(nh);

	NullUserTracking.UserID = -1;

	//init msg
	track_pub = nh.advertise<UserTracking>("peopleTracking", 100);
	std::cout << "init ok\n";

	//ciclo principale.
	while(nh.ok())
	{
		//aggiorna il contesto e aspetta
		g_Context.WaitAndUpdateAll();

		//pubblica le trasformazioni su frame_id
		publishTrackings(track_pub);

		if (VisualTrackingOn)
			TrackingVisualization_pub.publish(TrackingVisualizationImage);

		ros::spinOnce();

		//dormi per un tot.
		ros::Rate(30).sleep();
	}

	//rilascia risorse e esci.
	//g_Context.Shutdown();
	g_Context.Release();
	return EXIT_SUCCESS;
}


/*publishing----------------------------------------------------------------------------------------------*/

void publishTrackings(ros::Publisher& track_pub)
{
	XnUInt16 NITE_users_count = MAX_USERS;
	int real_users_count = 0;
	XnUserID users[MAX_USERS];
	UserTracking allTrackings;

	g_UserGenerator.GetUsers(users, NITE_users_count);

	//per ogni utente pubblica il tracking
	for(int i = 0; i < NITE_users_count; i++)
	{
		XnUserID CurrUser = users[i];
		SingleUserTracking thisUserPrevTracking;
		TrackingDenoiser thisUserTd(MOBILE_AVG_WINDOW_SIZE);
		try
		{
			thisUserPrevTracking = PrevTrackings.at(CurrUser);
			thisUserTd = TrackingDenoisers.at(CurrUser);
		}
		catch(std::out_of_range&)
		{
			thisUserPrevTracking = NullUserTracking;
		}
		SingleUserTracking sut = calcSingleUserTracking(CurrUser, thisUserPrevTracking, thisUserTd);

		if (sut.UserID == -1)
		{
			//std::cout << "user[" << i << "] è null. userCount = " << users_count << "\n";
			continue;
		}

		allTrackings.user.push_back(sut);
		real_users_count++;
		PrevTrackings[CurrUser] = thisUserPrevTracking;
		TrackingDenoisers[CurrUser] = thisUserTd;
	}
	allTrackings.userCount = real_users_count;

	if (VisualTrackingOn)
		VisualizeTracking(TrackingVisualizationMat, allTrackings);

	track_pub.publish(allTrackings);
}

void VisualizeTracking(cv_bridge::CvImage& CvbImage, people_tracker_msg::UserTracking trackings)
{
	double x, y, z;
	int Xshift = CvbImage.image.cols / 2;
	int Yshift = CvbImage.image.rows / 2;
	SingleUserTracking u;
	TrackedPoint p;
	for (int i = 0; i < trackings.user.size(); i++)
	{
		u = trackings.user[i];
		p.x = u.Pos_X;
		p.y = u.Pos_Y;
		p.z = u.Pos_Z;
		p = CameraLinkToCameraDepthOpticalFrame(p);

		//TODO: fix this, x,y,z have new meaning now.
		x = -p.x/p.z * CvbImage.image.cols/0.5543;
		y = -p.y/p.z * CvbImage.image.rows;
		z = 1 / p.z * 100;
		//std::cout << "x/z = " << p.y/p.z;
		//std::cout << "x =" << x << " y = " << y << " z = " << z;
		cv::circle(CvbImage.image, cv::Point(x+Xshift, y+Yshift), z, CV_RGB(200,50,200), 5);
	}

	//sensor_msgs::ImagePtr ptr =
	TrackingVisualizationImage = *CvbImage.toImageMsg(); // ptr ;
}


//calcola e pubblica posizione e velocità di user

SingleUserTracking calcSingleUserTracking(XnUserID const& user, SingleUserTracking& PrevTracking, TrackingDenoiser& td)
{

	//ottiene posizione e orientamento busto di user
	XnSkeletonJointPosition joint_position;

	if(g_UserGenerator.GetSkeletonCap().IsTracking(user))
	{
		//ottiene posizione e orientamento di testa e torso
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, WHAT_TO_TRACK, joint_position);

		if(joint_position.fConfidence < 1)
		{
			return NullUserTracking;
		}
	}
	else
	{
		return NullUserTracking;
	}
	TrackedPoint rawTrackedPoint;
	rawTrackedPoint.x = joint_position.position.X/1000;
	rawTrackedPoint.y = joint_position.position.Y/1000;
	rawTrackedPoint.z = joint_position.position.Z/1000;
	rawTrackedPoint = CameraDepthOpticalFrameToCameraLink(rawTrackedPoint);

	//filtra con media mobile
	td.addTracking(rawTrackedPoint);
	TrackedPoint p = td.getAvg();

	double xfiltered = p.x;
	double yfiltered = p.y;
	double zfiltered = p.z;

	//floor to mm. questi dati sono quelli pubblicati
	double xfilteredRounded = ApplyNewPrecision(xfiltered, min_position_precision);
	double yfilteredRounded = ApplyNewPrecision(yfiltered, min_position_precision);
	double zfilteredRounded = ApplyNewPrecision(zfiltered, min_position_precision);

	SingleUserTracking sut;

	sut.timestamp = ros::Time::now();
	sut.UserID = user;
	sut.Pos_X = xfilteredRounded;
	sut.Pos_Y = yfilteredRounded;
	sut.Pos_Z = zfilteredRounded;

	if (PrevTracking.UserID == -1)
	{
		//è il primo tracking per questo utente, nn ho info su stato precedente. parte "da fermo"
		sut.Vel_X = 0;
		sut.Vel_Y = 0;
		sut.Vel_Z = 0;
	}
	else
	{
		//std::cout << "ok \n";
		ros::Duration dur = sut.timestamp - PrevTracking.timestamp;
		double time_delay = dur.toSec();
		//calcola velocità tra istanti successivi
		double Vel_X = (xfiltered - PrevTracking.Pos_X) / time_delay;
		double Vel_Y = (yfiltered - PrevTracking.Pos_Y) / time_delay;
		double Vel_Z = (zfiltered - PrevTracking.Pos_Z) / time_delay;

		//std::cout << Vel_X << "\n";

		sut.Vel_X = applyThreshold(Vel_X, VELOCITY_THRESHOLD);
		sut.Vel_Y = applyThreshold(Vel_Y, VELOCITY_THRESHOLD);
		sut.Vel_Z = applyThreshold(Vel_Z, VELOCITY_THRESHOLD);

		//std::cout << "dopo thresh: " << Vel_X << "\n";
	}
	//write to PrevTracking i dati attuali (NON arrotondati ma filtrati)
	PrevTracking.UserID = sut.UserID;
	PrevTracking.timestamp = sut.timestamp;
	PrevTracking.Pos_X = xfiltered;
	PrevTracking.Pos_Y = yfiltered;
	PrevTracking.Pos_Z = zfiltered;
	return sut;
}

/*
 * depth_optical > camera_link
 * x > -y
 * y > -z
 * z > x
 * */
TrackedPoint CameraDepthOpticalFrameToCameraLink(TrackedPoint old)
{
	TrackedPoint newPoint;
	newPoint.x = old.z;
	newPoint.y = -old.x;
	newPoint.z = -old.y;
	//aggiusto offeset tra i due frames
	newPoint.y -= 0.02;
	return newPoint;
}

TrackedPoint CameraLinkToCameraDepthOpticalFrame(TrackedPoint old)
{
	TrackedPoint newPoint;
	newPoint.x = old.z;
	newPoint.y = -old.x;
	newPoint.z = -old.y;
	//aggiusto offeset tra i due frames
	newPoint.y += 0.02;
	return newPoint;
}

double applyThreshold(double num, double threshold)
{
	return fabs(num) >= threshold ? num : 0;
}

double ApplyNewPrecision(double num, double LeastSignificantChange)
{
	return floor(num/LeastSignificantChange)*LeastSignificantChange;
}

/*callbacks---------------------------------------------------------------------------------------------*/

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	//std::cout << "newUser\n";
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
	std::string configFilename = ros::package::getPath("people_tracker") + "/init/openni_tracker.xml";
	genericUserCalibrationFileName = ros::package::getPath("people_tracker") + "/init/GenericUserCalibration.bin";
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

	//tracking visualization-------------
	subImage = nh.subscribe("/camera/rgb/image_color", 10, updateRgbImage);
	TrackingVisualization_pub = nh.advertise<sensor_msgs::Image>("peopleTrackerVisualization", 10);
}


void updateRgbImage(const sensor_msgs::Image& rgbImageMsg)
{
	TrackingVisualizationMat = *cv_bridge::toCvCopy(rgbImageMsg);
}

void updateDepthImage(const sensor_msgs::Image& depthImageMsg)
{
	currDepthImage = depthImageMsg;
}
