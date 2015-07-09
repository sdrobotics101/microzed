/*
 * Cubeception.cpp
 *
 *  Created on: Jul 7, 2015
 *      Author: Rahul
 */

#include "Cubeception.h"

Cubeception::Cubeception(std::string configFile) {
	_iniReader = new INIReader(configFile);

	initNetworkClient();
	initIMUController();
	initPSController();
	initPWMController();
}

Cubeception::~Cubeception() {
	delete _iniReader;
	delete _networkClient;
	delete _imuController;
	delete _psController;
	delete _pwmController;
}

void Cubeception::start() {
	_networkClient->start();
	_imuController->start();
	_psController->start();
	_pwmController->start();
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
	_pwmController = new PWMController(_networkClient,
									   _imuController,
									   _psController,
									   _iniReader->GetInteger("pwm", "pwmaddr", 0x00010000),
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
