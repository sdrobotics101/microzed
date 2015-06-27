/*
 * IMUController.cpp
 *
 *  Created on: Jun 26, 2015
 *      Author: Rahul
 */

#include "IMUController.h"

IMUController::IMUController() {
	initSensors();
	reset();
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
	_avgAX = 0;
	_avgAY = 0;
	_avgAZ = 0;
	_avgGX = 0;
	_avgGY = 0;
	_avgGZ = 0;
	_avgMX = 0;
	_avgMY = 0;
	_avgMZ = 0;

	_xAngle = 0;
	_yAngle = 0;
	_zAngle = 0;
}

double IMUController::getXRotation() {
	return _xAngle;
}

double IMUController::getYRotation() {
	return _yAngle;
}

double IMUController::getZRotation() {
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

}

void IMUController::calculateOrientation() {

}

void IMUController::run() {
	while(1) {
		pollSensors();
		calculateOrientation();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}
