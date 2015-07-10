/*
 * Cubeception.cpp
 *
 *  Created on: Jul 7, 2015
 *      Author: Rahul
 */

#include "Cubeception.h"

Cubeception::Cubeception(std::string configFile) {
	_iniReader = new INIReader(configFile);
	if (_iniReader->ParseError() < 0) {
		std::cout << "Could not find config file" << std::endl;
		std::cout << "Initializing using default values" << std::endl;
	} else {
		std::cout << "Found config file" << std::endl;
	}
	std::cout << "Beginning Initialization" << std::endl;

	initNetworkClient();
	std::cout << "Initialized Network Client" << std::endl;

	initIMUController();
	std::cout << "Initialized IMU Controller" << std::endl;

	initPSController();
	std::cout <<"Initialized PS Controller" << std::endl;

	initPWMController();
	std::cout << "Initialized PWM Controller" << std::endl;

	std::cout << "Finished Initialization" << std::endl;
}

Cubeception::~Cubeception() {
	delete _iniReader;
	delete _networkClient;
	delete _imuController;
	delete _psController;
	delete _pwmController;
}

void Cubeception::start() {
	std::cout << "Starting" << std::endl;

	_networkClient->start();
	std::cout << "Network Client Started" << std::endl;

	_imuController->start();
	std::cout << "IMU Controller Started" << std::endl;

	_psController->start();
	std::cout << "PS Controller Started" << std::endl;

	_pwmController->start();
	std::cout << "PWM Controller Started" << std::endl;

	std::cout << "Completed Startup" << std::endl;
}

void Cubeception::initNetworkClient() {
	_networkClient = new NetworkClient();
	_networkClient->open(_iniReader->Get("network", "ipaddr", "192.168.1.21"),
						 _iniReader->GetInteger("network", "port", 8888));
}

void Cubeception::initIMUController() {
	Eigen::Vector3d mpu0AccBias;
	Eigen::Vector3d mpu1AccBias;
	Eigen::Vector3d mpu0GyroBias;
	Eigen::Vector3d mpu1GyroBias;
	Eigen::Vector3d mpu0MagBias;
	Eigen::Vector3d mpu1MagBias;
	Eigen::Matrix3d mpu0MagTransform;
	Eigen::Matrix3d mpu1MagTransform;

	mpu0AccBias(XAXIS) = _iniReader->GetReal("imu", "mpu0accbiasx", 0);
	mpu0AccBias(YAXIS) = _iniReader->GetReal("imu", "mpu0accbiasy", 0);
	mpu0AccBias(ZAXIS) = _iniReader->GetReal("imu", "mpu0accbiasz", 0);

	mpu1AccBias(XAXIS) = _iniReader->GetReal("imu", "mpu1accbiasx", 0);
	mpu1AccBias(YAXIS) = _iniReader->GetReal("imu", "mpu1accbiasy", 0);
	mpu1AccBias(ZAXIS) = _iniReader->GetReal("imu", "mpu1accbiasz", 0);

	mpu0GyroBias(XAXIS) = _iniReader->GetReal("imu", "mpu0gyrobiasx", 0);
	mpu0GyroBias(YAXIS) = _iniReader->GetReal("imu", "mpu0gyrobiasy", 0);
	mpu0GyroBias(ZAXIS) = _iniReader->GetReal("imu", "mpu0gyrobiasz", 0);

	mpu1GyroBias(XAXIS) = _iniReader->GetReal("imu", "mpu1gyrobiasx", 0);
	mpu1GyroBias(YAXIS) = _iniReader->GetReal("imu", "mpu1gyrobiasy", 0);
	mpu1GyroBias(ZAXIS) = _iniReader->GetReal("imu", "mpu1gyrobiasz", 0);

	mpu0MagBias(XAXIS) = _iniReader->GetReal("imu", "mpu0magbiasx", 0);
	mpu0MagBias(YAXIS) = _iniReader->GetReal("imu", "mpu0magbiasy", 0);
	mpu0MagBias(ZAXIS) = _iniReader->GetReal("imu", "mpu0magbiasz", 0);

	mpu1MagBias(XAXIS) = _iniReader->GetReal("imu", "mpu1magbiasx", 0);
	mpu1MagBias(YAXIS) = _iniReader->GetReal("imu", "mpu1magbiasy", 0);
	mpu1MagBias(ZAXIS) = _iniReader->GetReal("imu", "mpu1magbiasz", 0);

	mpu0MagTransform(0,0) = _iniReader->GetReal("imu", "mpu0magtransform00", 0);
	mpu0MagTransform(0,1) = _iniReader->GetReal("imu", "mpu0magtransform01", 0);
	mpu0MagTransform(0,2) = _iniReader->GetReal("imu", "mpu0magtransform02", 0);
	mpu0MagTransform(1,0) = _iniReader->GetReal("imu", "mpu0magtransform10", 0);
	mpu0MagTransform(1,1) = _iniReader->GetReal("imu", "mpu0magtransform11", 0);
	mpu0MagTransform(1,2) = _iniReader->GetReal("imu", "mpu0magtransform12", 0);
	mpu0MagTransform(2,0) = _iniReader->GetReal("imu", "mpu0magtransform20", 0);
	mpu0MagTransform(2,1) = _iniReader->GetReal("imu", "mpu0magtransform21", 0);
	mpu0MagTransform(2,2) = _iniReader->GetReal("imu", "mpu0magtransform22", 0);

	mpu1MagTransform(0,0) = _iniReader->GetReal("imu", "mpu1magtransform00", 0);
	mpu1MagTransform(0,1) = _iniReader->GetReal("imu", "mpu1magtransform01", 0);
	mpu1MagTransform(0,2) = _iniReader->GetReal("imu", "mpu1magtransform02", 0);
	mpu1MagTransform(1,0) = _iniReader->GetReal("imu", "mpu1magtransform10", 0);
	mpu1MagTransform(1,1) = _iniReader->GetReal("imu", "mpu1magtransform11", 0);
	mpu1MagTransform(1,2) = _iniReader->GetReal("imu", "mpu1magtransform12", 0);
	mpu1MagTransform(2,0) = _iniReader->GetReal("imu", "mpu1magtransform20", 0);
	mpu1MagTransform(2,1) = _iniReader->GetReal("imu", "mpu1magtransform21", 0);
	mpu1MagTransform(2,2) = _iniReader->GetReal("imu", "mpu1magtransform22", 0);

	_imuController = new IMUController(_iniReader->GetInteger("imu", "mpu0addr", 0x00040000),
									   _iniReader->GetInteger("imu", "mpu1addr", 0x00050000),
									   _iniReader->GetReal("imu", "combine", 0.8),
									   mpu0AccBias,
									   mpu1AccBias,
									   mpu0GyroBias,
									   mpu1GyroBias,
									   mpu0MagBias,
									   mpu1MagBias,
									   mpu0MagTransform,
									   mpu1MagTransform);
}

void Cubeception::initPSController() {
	_psController = new PSController(_iniReader->GetInteger("ps", "ms0addr", 0x00020000),
									 _iniReader->GetInteger("ps", "ms1addr", 0x00030000),
									 _iniReader->GetReal("ps", "waterdensity", 1000),
									 _iniReader->GetReal("ps", "atmosphericpressure", 1000));
}

void Cubeception::initPWMController() {
	uint32_t pwmMap[24];
	pwmMap[MXF1] = _iniReader->GetInteger("map", "mxf1", 0);
	pwmMap[MXF2] = _iniReader->GetInteger("map", "mxf2", 1);
	pwmMap[MXF3] = _iniReader->GetInteger("map", "mxf3", 2);
	pwmMap[MXF4] = _iniReader->GetInteger("map", "mxf4", 3);

	pwmMap[MXR1] = _iniReader->GetInteger("map", "mxr1", 4);
	pwmMap[MXR2] = _iniReader->GetInteger("map", "mxr2", 5);
	pwmMap[MXR3] = _iniReader->GetInteger("map", "mxr3", 6);
	pwmMap[MXR4] = _iniReader->GetInteger("map", "mxr4", 7);

	pwmMap[MYF1] = _iniReader->GetInteger("map", "myf1", 8);
	pwmMap[MYF2] = _iniReader->GetInteger("map", "myf2", 9);
	pwmMap[MYF3] = _iniReader->GetInteger("map", "myf3", 10);
	pwmMap[MYF4] = _iniReader->GetInteger("map", "myf4", 11);

	pwmMap[MYR1] = _iniReader->GetInteger("map", "myr1", 12);
	pwmMap[MYR2] = _iniReader->GetInteger("map", "myr2", 13);
	pwmMap[MYR3] = _iniReader->GetInteger("map", "myr3", 14);
	pwmMap[MYR4] = _iniReader->GetInteger("map", "myr4", 15);

	pwmMap[MZF1] = _iniReader->GetInteger("map", "mzf1", 16);
	pwmMap[MZF2] = _iniReader->GetInteger("map", "mzf2", 17);
	pwmMap[MZF3] = _iniReader->GetInteger("map", "mzf3", 18);
	pwmMap[MZF4] = _iniReader->GetInteger("map", "mzf4", 19);

	pwmMap[MZR1] = _iniReader->GetInteger("map", "mzr1", 20);
	pwmMap[MZR2] = _iniReader->GetInteger("map", "mzr2", 21);
	pwmMap[MZR3] = _iniReader->GetInteger("map", "mzr3", 22);
	pwmMap[MZR4] = _iniReader->GetInteger("map", "mzr4", 23);

	_pwmController = new PWMController(_networkClient,
									   _imuController,
									   _psController,
									   _iniReader->GetInteger("pwm", "pwmaddr", 0x00010000),
									   pwmMap,
									   _iniReader->GetReal("pwm", "combine", 0.6),
									   _iniReader->GetReal("pwm", "xp", 0),
									   _iniReader->GetReal("pwm", "xi", 0),
									   _iniReader->GetReal("pwm", "xd", 0),
									   _iniReader->GetReal("pwm", "xf", 0),
									   _iniReader->GetReal("pwm", "yp", 0),
									   _iniReader->GetReal("pwm", "yi", 0),
									   _iniReader->GetReal("pwm", "yd", 0),
									   _iniReader->GetReal("pwm", "yf", 0),
									   _iniReader->GetReal("pwm", "zp", 0),
									   _iniReader->GetReal("pwm", "zi", 0),
									   _iniReader->GetReal("pwm", "zd", 0),
									   _iniReader->GetReal("pwm", "zf", 0),
									   _iniReader->GetReal("pwm", "dp", 0),
									   _iniReader->GetReal("pwm", "di", 0),
									   _iniReader->GetReal("pwm", "dd", 0),
									   _iniReader->GetReal("pwm", "df", 0));
}
