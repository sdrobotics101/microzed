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

#define MPU0TRANSFORM00 1.943
#define MPU0TRANSFORM01 -0.014
#define MPU0TRANSFORM02 0.141
#define MPU0TRANSFORM10 -0.207
#define MPU0TRANSFORM11 2.055
#define MPU0TRANSFORM12 -0.039
#define MPU0TRANSFORM20 -0.046
#define MPU0TRANSFORM21 0.073
#define MPU0TRANSFORM22 2.016

#define MPU1TRANSFORM00 1.943
#define MPU1TRANSFORM01 0.019
#define MPU1TRANSFORM02 0.003
#define MPU1TRANSFORM10 -0.121
#define MPU1TRANSFORM11 2.063
#define MPU1TRANSFORM12 0.031
#define MPU1TRANSFORM20 0.04
#define MPU1TRANSFORM21 0.082
#define MPU1TRANSFORM22 2.152

#define MPU0MAGXBIAS 252.757
#define MPU0MAGYBIAS 109.683
#define MPU0MAGZBIAS -176.555

#define MPU1MAGXBIAS -279.989
#define MPU1MAGYBIAS 60.419
#define MPU1MAGZBIAS -334.917

#define MPU0GYROXBIAS -0.131603
#define MPU0GYROYBIAS 0.374885
#define MPU0GYROZBIAS 0.79771

#define MPU1GYROXBIAS -0.0870992
#define MPU1GYROYBIAS 0.425496
#define MPU1GYROZBIAS -2.7074

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

class IMUController {
public:
	IMUController(uint32_t mpu0Addr,
				  uint32_t mpu1Addr,
				  double combine,
				  Eigen::Vector3d mpu0AccBias,
				  Eigen::Vector3d mpu1AccBias,
				  Eigen::Vector3d mpu0GyroBias,
				  Eigen::Vector3d mpu1GyroBias,
				  Eigen::Vector3d mpu0MagBias,
				  Eigen::Vector3d mpu1MagBias,
				  Eigen::Matrix3d mpu0MagTransform,
				  Eigen::Matrix3d mpu1MagTransform);
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

	const uint32_t _mpu0Addr;
	const uint32_t _mpu1Addr;
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

	Eigen::Vector3d _mpu0AccBias;
	Eigen::Vector3d _mpu1AccBias;

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
	double _combine;

	Timer _timer;

	std::thread *_thread;
	std::mutex _mutex;
};

#endif /* IMUCONTROLLER_H_ */
