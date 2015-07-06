/*
 * IMUController.h
 *
 *  Created on: Jun 26, 2015
 *      Author: Rahul
 */

#ifndef IMUCONTROLLER_H_
#define IMUCONTROLLER_H_

#include <thread>
#include <mutex>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>

#include "mpu9250.h"
#include "../Utilities/Timer.h"

#define MPU0ADDR 0x00040000
#define MPU1ADDR 0x00050000

#define COMBINE 0.5

#define MPU0TRANSFORM00 0
#define MPU0TRANSFORM01 0
#define MPU0TRANSFORM02 0
#define MPU0TRANSFORM10 0
#define MPU0TRANSFORM11 0
#define MPU0TRANSFORM12 0
#define MPU0TRANSFORM20 0
#define MPU0TRANSFORM21 0
#define MPU0TRANSFORM22 0

#define MPU1TRANSFORM00 0
#define MPU1TRANSFORM01 0
#define MPU1TRANSFORM02 0
#define MPU1TRANSFORM10 0
#define MPU1TRANSFORM11 0
#define MPU1TRANSFORM12 0
#define MPU1TRANSFORM20 0
#define MPU1TRANSFORM21 0
#define MPU1TRANSFORM22 0

#define MPU0MAGXBIAS 0
#define MPU0MAGYBIAS 0
#define MPU0MAGZBIAS 0

#define MPU1MAGXBIAS 0
#define MPU1MAGYBIAS 0
#define MPU1MAGZBIAS 0

#define MPU0GYROXBIAS 0
#define MPU0GYROYBIAS 0
#define MPU0GYROZBIAS 0

#define MPU1GYROXBIAS 0
#define MPU1GYROYBIAS 0
#define MPU1GYROZBIAS 0

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

class IMUController {
public:
	IMUController();
	virtual ~IMUController();

	void start();
	void reset();

	double getXRotation();
	double getYRotation();
	double getZRotation();

	MPU9250 *getMPU0();
	MPU9250 *getMPU1();

private:
	void initSensors();
	void pollSensors();
	void correctData();
	void calculateOrientation();

	void run();

	MPU9250 *_mpu0;
	MPU9250 *_mpu1;

	Eigen::Vector3d _mpu0AccData;
	Eigen::Vector3d _mpu0GyroData;
	Eigen::Vector3d _mpu0MagData;

	Eigen::Vector3d _mpu1AccData;
	Eigen::Vector3d _mpu1GyroData;
	Eigen::Vector3d _mpu1MagData;

	Eigen::Vector3d _avgAccData;
	Eigen::Vector3d _avgGyroData;
	Eigen::Vector3d _avgMagData;

	Eigen::Vector3d _mpu0GyroBias;
	Eigen::Vector3d _mpu1GyroBias;

	Eigen::Matrix3d _mpu0MagTransform;
	Eigen::Matrix3d _mpu1MagTransform;
	Eigen::Vector3d _mpu0MagBias;
	Eigen::Vector3d _mpu1MagBias;

	Eigen::Vector3d _cross1;
	Eigen::Vector3d _cross2;

	Eigen::Vector3d _accMagAngles;
	Eigen::Vector3d _gyroAngles;
	Eigen::Vector3d _combinedAngles;

	bool _useGyro;

	Timer _timer;

	std::thread *_thread;
	std::mutex _mutex;
};

#endif /* IMUCONTROLLER_H_ */
