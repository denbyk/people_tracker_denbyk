#include "pan_controller.hpp"

#define MAX_SPEED 0.8

PanController::PanController(ros::NodeHandle& nh): private_nh_("~")
{
	private_nh_.param("DeviceIndex", mDeviceIndex, 1);
	private_nh_.param("Baudnum", mBaudnum, 34);
	private_nh_.param("UpdateRate", mUpdateRate, 20);
	std::string tempYamlPath("/home/rob/virgil_ws/src/hostess_robot/hostess_skeleton_tracker/launch/include/pan_joint.yaml");
	private_nh_.param("YamlPath", mYamlPath, tempYamlPath);

	//verifico se il plugin può essere caricato

	try
	{
		dxio_loader.reset(new pluginlib::ClassLoader<dynamixelIO_wrapper_base::DynamixelIO_Base>("cyton_wrapper","dynamixelIO_wrapper_base::DynamixelIO_Base"));
	}
	catch(pluginlib::PluginlibException& e)
	{
		ROS_ERROR("The plugin_loader failed to load for some reason. Error: %s", e.what());
	}

	//creo un'istanza del plugin, così dxio potrà accedere ai metodi della classe DynamixelIO_Base nel namespace dynamixelIO_wrapper_base

	try
	{
		dxio = dxio_loader->createInstance("dynamixelIO_wrapper_plugin::DynamixelIO");
		ROS_INFO("Loading PlugIn IO...");
		dxio->initialize(mDeviceIndex, mBaudnum, mUpdateRate, mYamlPath, &nh);

		dxio->start();
	}
	catch(pluginlib::PluginlibException& e)
	{
		ROS_ERROR("The plugin_loader failed to load for some reason. Error: %s", e.what());
	}

	extremeRight = (dxio->getMinAngle(0) - 2048) * 0.0015339804;	//Estrema destra del motore
	extremeLeft = (dxio->getMaxAngle(0) - 2048) * 0.0015339804;	//Estrema sinistra del motore

	turningSpeed = 0;

	lambda = 0.98;

	goHome();

	while(!isHome())
	{

	}

	return;
}

PanController::~PanController()
{
	goHome();

	while(!isHome())
	{

	}

	dxio->stop();

	return;
}

void PanController::goHome()
{
	std::vector<std::vector<double> > v;
	std::vector<double> pv;

	pv.clear();
	pv.push_back(0.0);
	pv.push_back(MAX_SPEED);
	v.push_back(pv);

	dxio->setMultiPosVel(v);
}

bool PanController::isHome()
{
	if(getRotation() <= 0.01 && getRotation() >= -0.01)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void PanController::standStill()
{
	std::vector<std::vector<double> > v;
	std::vector<double> pv;

	double presentPosition;

	dxio->getPresentPosition(0, presentPosition);

	turningSpeed = 0.0;

	pv.clear();
	pv.push_back(presentPosition);
	pv.push_back(turningSpeed);
	v.push_back(pv);

	dxio->setMultiPosVel(v);

	return;
}

void PanController::turn(double angle, double robotRotation)
{
	double presentPosition;

	dxio->getPresentPosition(0, presentPosition);

	std::vector<std::vector<double> > v;
	std::vector<double> pv;

	double deltaAngle = angle - robotRotation;

	targetPosition = presentPosition + deltaAngle;

	turningSpeed = ((1 - lambda) * turningSpeed) + (lambda * fabs((2.2 * angle) - robotRotation));

	pv.clear();
	pv.push_back(targetPosition);
	pv.push_back(std::min(MAX_SPEED, turningSpeed));
	v.push_back(pv);

	dxio->setMultiPosVel(v);
}

void PanController::continueTurning()
{
	double presentPosition;

	dxio->getPresentPosition(0, presentPosition);

	if(presentPosition >= targetPosition - 0.01 && presentPosition >= targetPosition + 0.01)
	{
		return;
	}
	else
	{
		turningSpeed = std::max(0.0, turningSpeed - 0.005);
		std::vector<std::vector<double> > v;
		std::vector<double> pv;

		pv.clear();
		pv.push_back(targetPosition);
		pv.push_back(std::min(MAX_SPEED, turningSpeed));
		v.push_back(pv);

		dxio->setMultiPosVel(v);
	}
}

void PanController::turnLeft(double speed)
{
	std::vector<std::vector<double> > v;
	std::vector<double> pv;

	pv.clear();
	pv.push_back(extremeLeft);
	pv.push_back(std::min(MAX_SPEED, speed));
	v.push_back(pv);

	dxio->setMultiPosVel(v);

	return;
}

void PanController::turnRight(double speed)
{
	std::vector<std::vector<double> > v;
	std::vector<double> pv;

	pv.clear();
	pv.push_back(extremeRight);
	pv.push_back(std::min(MAX_SPEED, speed));
	v.push_back(pv);

	dxio->setMultiPosVel(v);

	return;
}

double PanController::getRotation()
{
	double presentPosition;

	dxio->getPresentPosition(0, presentPosition);

	return presentPosition;
}
