/*
 * IMUController.cpp
 *
 *  Created on: Jun 26, 2015
 *      Author: Rahul
 */

#include "IMUController.h"

IMUController::IMUController() {
	initSensors();
	for (int i = 0;i < 10;i++) {
			_mpu0Data[i] = 0;
			_mpu1Data[i] = 0;
			_avgData[i] = 0;
	}

	_acc << 0,0,0;
	_mag << 0,0,0;
	_cross1 << 0,0,0;
	_cross2 << 0,0,0;

	_xAngle = 0;
	_yAngle = 0;
	_zAngle = 0;
}

IMUController::~IMUController() {
	_thread->join();
	delete _mpu0;
	delete _mpu1;
}

void IMUController::start() {
	_thread = new std::thread(&IMUController::run, this);
}

void IMUController::reset() {
	std::lock_guard<std::mutex> lock(_mutex);
	for (int i = 0;i < 10;i++) {
		_mpu0Data[i] = 0;
		_mpu1Data[i] = 0;
		_avgData[i] = 0;
	}

	_xAngle = 0;
	_yAngle = 0;
	_zAngle = 0;
}

double IMUController::getXRotation() {
	std::lock_guard<std::mutex> lock(_mutex);
	return _xAngle;
}

double IMUController::getYRotation() {
	std::lock_guard<std::mutex> lock(_mutex);
	return _yAngle;
}

double IMUController::getZRotation() {
	std::lock_guard<std::mutex> lock(_mutex);
	return _zAngle;
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
	_mpu0->read(&_mpu0Data[0], &_mpu0Data[1], &_mpu0Data[2],
				&_mpu0Data[3], &_mpu0Data[4], &_mpu0Data[5],
				&_mpu0Data[6], &_mpu0Data[7], &_mpu0Data[8],
				&_mpu0Data[9]);
	_mpu1->read(&_mpu1Data[0], &_mpu1Data[1], &_mpu1Data[2],
				&_mpu1Data[3], &_mpu1Data[4], &_mpu1Data[5],
				&_mpu1Data[6], &_mpu1Data[7], &_mpu1Data[8],
				&_mpu1Data[9]);
	for (int i = 0;i < 10;i++) {
		_avgData[i] = ((_mpu0Data[i] + _mpu1Data[i]) / 2);
	}
}

void IMUController::calculateOrientation() {
	std::lock_guard<std::mutex> lock(_mutex);
	_acc << _avgData[0], _avgData[1], _avgData[2];
	_mag << _mpu0Data[6], _mpu0Data[7], _mpu0Data[8];
	_cross1 = _acc.cross(_mag);
	_cross2 = _cross1.cross(_acc);

	_acc.normalize();
	_cross1.normalize();
	_cross2.normalize();

	if (_cross2[2] != 1 && _cross2[2] != -1) {
		_yAngle = asin(-_cross2[2]);
		_xAngle = asin(_cross1[2] / cos(_yAngle));
		_zAngle = asin(_cross2[1] / cos(_yAngle));

		_xAngle *= 180/M_PI;
		_yAngle *= 180/M_PI;
		_zAngle *= 180/M_PI;
	}
}

void IMUController::run() {
	while(1) {
		pollSensors();
		calculateOrientation();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}
