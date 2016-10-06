#ifndef PAN_CONTROLLER_HPP_
#define PAN_CONTROLLER_HPP_

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <cyton_wrapper/dynamixelIO_wrapper_base.h>

class PanController
{
	private:
		ros::NodeHandle private_nh_;

		boost::shared_ptr<pluginlib::ClassLoader<dynamixelIO_wrapper_base::DynamixelIO_Base> > dxio_loader;

		boost::shared_ptr<dynamixelIO_wrapper_base::DynamixelIO_Base> dxio;

		int mDeviceIndex, mBaudnum, mUpdateRate;
		std::string mYamlPath;

		double extremeLeft, extremeRight;

		bool homed = false;

		double turningSpeed, lambda, targetPosition;

	public:
		PanController(ros::NodeHandle&);
		~PanController();
		void goHome();
		bool isHome();
		void standStill();
		void turn(double, double);
		void turnRight(double);
		void turnLeft(double);
		void continueTurning();
		double getRotation();
};

#endif /* PAN_CONTROLLER_HPP_ */
