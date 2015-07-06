/*
 * IMUController.cpp
 *
 *  Created on: Jun 26, 2015
 *      Author: Rahul
 */

#include "IMUController.h"

IMUController::IMUController() {
	initSensors();
	_mpu0GyroBias << MPU0GYROXBIAS, MPU0GYROYBIAS, MPU0GYROZBIAS;
	_mpu1GyroBias << MPU1GYROXBIAS, MPU1GYROYBIAS, MPU1GYROZBIAS;

	_mpu0MagTransform << MPU0TRANSFORM00, MPU0TRANSFORM01, MPU0TRANSFORM02,
						 MPU0TRANSFORM10, MPU0TRANSFORM11, MPU0TRANSFORM12,
						 MPU0TRANSFORM20, MPU0TRANSFORM21, MPU0TRANSFORM22;
	_mpu1MagTransform << MPU1TRANSFORM00, MPU1TRANSFORM01, MPU1TRANSFORM02,
						 MPU1TRANSFORM10, MPU1TRANSFORM11, MPU1TRANSFORM12,
					     MPU1TRANSFORM20, MPU1TRANSFORM21, MPU1TRANSFORM22;

	_mpu0MagBias << MPU0MAGXBIAS, MPU0MAGYBIAS, MPU0MAGZBIAS;
	_mpu1MagBias << MPU1MAGXBIAS, MPU1MAGYBIAS, MPU1MAGZBIAS;

	_mpu0AccData.Zero();
	_mpu0GyroData.Zero();
	_mpu0GyroData.Zero();

	_mpu1AccData.Zero();
	_mpu1GyroData.Zero();
	_mpu1GyroData.Zero();

	_avgAccData.Zero();
	_avgGyroData.Zero();
	_avgMagData.Zero();

	_cross1.Zero();
	_cross2.Zero();

	_accMagAngles.Zero();
	_gyroAngles.Zero();
	_combinedAngles.Zero();

	_useGyro = false;
}

IMUController::~IMUController() {
	_thread->join();
	delete _mpu0;
	delete _mpu1;
}

void IMUController::start() {
	_thread = new std::thread(&IMUController::run, this);
	_timer.start();
}

void IMUController::reset() {
	std::lock_guard<std::mutex> lock(_mutex);
	_mpu0AccData.Zero();
	_mpu0GyroData.Zero();
	_mpu0GyroData.Zero();

	_mpu1AccData.Zero();
	_mpu1GyroData.Zero();
	_mpu1GyroData.Zero();

	_avgAccData.Zero();
	_avgGyroData.Zero();
	_avgMagData.Zero();

	_cross1.Zero();
	_cross2.Zero();

	_accMagAngles.Zero();
	_gyroAngles.Zero();
	_combinedAngles.Zero();

	_useGyro = false;
}

double IMUController::getXRotation() {
	std::lock_guard<std::mutex> lock(_mutex);
	return _combinedAngles(XAXIS);
}

double IMUController::getYRotation() {
	std::lock_guard<std::mutex> lock(_mutex);
	return _combinedAngles(YAXIS);
}

double IMUController::getZRotation() {
	std::lock_guard<std::mutex> lock(_mutex);
	return _combinedAngles(ZAXIS);
}

MPU9250* IMUController::getMPU0() {
	return _mpu0;
}

MPU9250* IMUController::getMPU1() {
	return _mpu1;
}

void IMUController::initSensors() {
	_mpu0 = new MPU9250(MPU0ADDR);
	_mpu1 = new MPU9250(MPU1ADDR);

	_mpu0->init(0,0);
	_mpu1->init(0,0);
}

void IMUController::pollSensors() {
	std::lock_guard<std::mutex> lock(_mutex);
	_mpu0->read_robot(&_mpu0AccData(XAXIS),  &_mpu0AccData(YAXIS),  &_mpu0AccData(ZAXIS),
					  &_mpu0GyroData(XAXIS), &_mpu0GyroData(YAXIS), &_mpu0GyroData(ZAXIS),
					  &_mpu0MagData(XAXIS),  &_mpu0MagData(YAXIS),  &_mpu0MagData(ZAXIS));

	_mpu1->read_robot(&_mpu1AccData(XAXIS),  &_mpu1AccData(YAXIS),  &_mpu1AccData(ZAXIS),
					  &_mpu1GyroData(XAXIS), &_mpu1GyroData(YAXIS), &_mpu1GyroData(ZAXIS),
					  &_mpu1MagData(XAXIS),  &_mpu1MagData(YAXIS),  &_mpu1MagData(ZAXIS));
}

void IMUController::correctData() {
	_avgAccData = _mpu0AccData + _mpu1AccData;
	_avgAccData /= 2;

	_mpu0GyroData -= _mpu0GyroBias;
	_mpu1GyroData -= _mpu1GyroBias;
	_avgGyroData = _mpu0GyroData + _mpu1GyroData;
	_avgGyroData /= 2;

	_mpu0MagData -= _mpu0MagBias;
	_mpu1MagData -= _mpu1MagBias;
	_mpu0MagData = _mpu0MagTransform * _mpu0MagData;
	_mpu1MagData = _mpu1MagTransform * _mpu1MagData;

	_avgMagData = _mpu0MagData + _mpu1MagData;
	_avgMagData /= 2;
}

void IMUController::calculateOrientation() {
	std::lock_guard<std::mutex> lock(_mutex);
	_cross1 = _avgAccData.cross(_avgMagData);
	_cross2 = _cross1.cross(_avgAccData);

	_avgAccData.normalize();
	_cross1.normalize();
	_cross2.normalize();

	if (_cross2(2) != 1 && _cross2(2) != -1) {
		_accMagAngles(YAXIS) = asin(-_cross2(2));
		_accMagAngles(XAXIS) = atan2((_cross1(2) / cos(_accMagAngles(YAXIS))),
									 (_avgAccData(2) / cos(_accMagAngles(YAXIS))));
		_accMagAngles(ZAXIS) = atan2((_cross2(1) / cos(_accMagAngles(YAXIS))),
									 (_cross2(0) / cos(_accMagAngles(YAXIS))));

		_accMagAngles *= (180/M_PI);
	}

	if (_useGyro) {
		_gyroAngles = _combinedAngles + (_avgGyroData * _timer.dt());
		_combinedAngles = (COMBINE * _gyroAngles) + ((1-COMBINE) * _accMagAngles);
	} else {
		_combinedAngles = _accMagAngles;
		_useGyro = true;
	}
}

void IMUController::run() {
	while(1) {
		pollSensors();
		correctData();
		calculateOrientation();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}
