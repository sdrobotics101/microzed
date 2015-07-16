/*
 * IMUController.cpp
 *
 *  Created on: Jun 26, 2015
 *      Author: Rahul
 */

#include "IMUController.h"

IMUController::IMUController(uint32_t mpu0Addr,
		  	  	  	  	  	 uint32_t mpu1Addr,
		  	  	  	  	  	 double combine,
		  	  	  	  	  	 Eigen::Vector3d mpu0AccBias,
		  	  	  	  	  	 Eigen::Vector3d mpu1AccBias,
		  	  	  	  	  	 Eigen::Vector3d mpu0GyroBias,
		  	  	  	  	  	 Eigen::Vector3d mpu1GyroBias,
		  	  	  	  	  	 Eigen::Vector3d mpu0MagBias,
		  	  	  	  	  	 Eigen::Vector3d mpu1MagBias,
		  	  	  	  	  	 Eigen::Matrix3d mpu0MagTransform,
		  	  	  	  	  	 Eigen::Matrix3d mpu1MagTransform) :
		  	  	  	  	  	 _mpu0Addr(mpu0Addr),
		  	  	  	  	  	 _mpu1Addr(mpu1Addr),
		  	  	  	  	  	 _combine(combine),
		  	  	  	  	  	 _mpu0AccBias(mpu0AccBias),
		  	  	  	  	  	 _mpu1AccBias(mpu1AccBias),
		  	  	  	  	  	 _mpu0GyroBias(mpu0GyroBias),
		  	  	  	  	  	 _mpu1GyroBias(mpu1GyroBias),
		  	  	  	  	  	 _mpu0MagBias(mpu0MagBias),
		  	  	  	  	  	 _mpu1MagBias(mpu1MagBias),
		  	  	  	  	  	 _mpu0MagTransform(mpu0MagTransform),
		  	  	  	  	  	 _mpu1MagTransform(mpu1MagTransform) {
	initSensors();
	_isThreadRunning = false;
	reset();
	std::cout << "IMUController initialized" << std::endl;
}

IMUController::~IMUController() {
	_thread->join();
	delete _mpu0;
	delete _mpu1;
}

void IMUController::start() {
	_isThreadRunning = true;
	_thread = new std::thread(&IMUController::run, this);
}

void IMUController::reset() {
	if (_isThreadRunning) {
		std::lock_guard<std::mutex> lock(_mutex);
	}
	_mpu0AccData.Zero();
	_mpu0GyroData.Zero();
	_mpu0MagData.Zero();

	_mpu1AccData.Zero();
	_mpu1GyroData.Zero();
	_mpu1MagData.Zero();

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

bool IMUController::isRunning() {
	return _isThreadRunning;
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
	_mpu0 = new MPU9250(_mpu0Addr);
	_mpu1 = new MPU9250(_mpu1Addr);

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
	if (_mpu0AccData(XAXIS) == 0 &&
		_mpu0AccData(YAXIS) == 0 &&
		_mpu0AccData(ZAXIS) == 0) {
		_mpu1AccData -= _mpu1AccBias;
		_avgAccData = _mpu1AccData;
	} else if (_mpu1AccData(XAXIS) == 0 &&
			   _mpu1AccData(YAXIS) == 0 &&
			   _mpu1AccData(ZAXIS) == 0) {
		_mpu0AccData -= _mpu0AccBias;
		_avgAccData = _mpu0AccData;
	} else {
		_mpu0AccData -= _mpu0AccBias;
		_mpu1AccData -= _mpu1AccBias;
		_avgAccData = _mpu0AccData + _mpu1AccData;
		_avgAccData /= 2;
	}

	if (_mpu0GyroData(XAXIS) == 0 &&
		_mpu0GyroData(YAXIS) == 0 &&
		_mpu0GyroData(ZAXIS) == 0) {
		_mpu1GyroData -= _mpu1GyroBias;
		_avgGyroData = _mpu1GyroData;
	} else if (_mpu1GyroData(XAXIS) == 0 &&
			   _mpu1GyroData(YAXIS) == 0 &&
			   _mpu1GyroData(ZAXIS) == 0) {
		_mpu0GyroData -= _mpu0GyroBias;
		_avgGyroData = _mpu0GyroData;
	} else {
		_mpu0GyroData -= _mpu0GyroBias;
		_mpu1GyroData -= _mpu1GyroBias;
		_avgGyroData = _mpu0GyroData + _mpu1GyroData;
		_avgGyroData /= 2;
	}

	if (_mpu0MagData(XAXIS) == 0 &&
		_mpu0MagData(YAXIS) == 0 &&
		_mpu0MagData(ZAXIS) == 0) {
		_mpu1MagData -= (_mpu1MagBias * MAGNETOMETERSCALEFACTOR);
		_mpu1MagData = _mpu1MagTransform * _mpu1MagData;
		_avgMagData = _mpu1MagData;
	} else if (_mpu1MagData(XAXIS) == 0 &&
			   _mpu1MagData(YAXIS) == 0 &&
			   _mpu1MagData(ZAXIS) == 0) {
		_mpu0MagData -= (_mpu0MagBias * MAGNETOMETERSCALEFACTOR);
		_mpu0MagData = _mpu0MagTransform * _mpu0MagData;
		_avgMagData = _mpu0MagData;
	} else {
		_mpu0MagData -= (_mpu0MagBias * MAGNETOMETERSCALEFACTOR);
		_mpu1MagData -= (_mpu1MagBias * MAGNETOMETERSCALEFACTOR);
		_mpu0MagData = _mpu0MagTransform * _mpu0MagData;
		_mpu1MagData = _mpu1MagTransform * _mpu1MagData;

		_avgMagData = _mpu0MagData + _mpu1MagData;
		_avgMagData /= 2;
	}
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

		_accMagAngles *= (-180/M_PI);
		_accMagAngles(XAXIS) += 180;
		if (_accMagAngles(XAXIS) > 180) {
			_accMagAngles(XAXIS) -= 360;
		}
	}

	if (_useGyro) {
		_gyroAngles = _combinedAngles + (_avgGyroData * _timer.dt());
		_combinedAngles = (_combine * _gyroAngles) + ((1-_combine) * _accMagAngles);
	} else {
		_combinedAngles = _accMagAngles;
		_useGyro = true;
	}
}

void IMUController::run() {
	_timer.start();
	std::cout << "IMUController started" << std::endl;
	while(1) {
		pollSensors();
		correctData();
		calculateOrientation();
		std::this_thread::sleep_for(std::chrono::milliseconds(IMULOOPTIME));
	}
}
