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
#include "../Utilities/Constants.h"

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
	bool isRunning();

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
	bool _isThreadRunning;
};

#endif /* IMUCONTROLLER_H_ */
