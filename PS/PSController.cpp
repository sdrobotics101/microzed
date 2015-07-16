/*
 * PSController.cpp
 *
 *  Created on: Jun 26, 2015
 *      Author: Rahul
 */

#include "PSController.h"

PSController::PSController(uint32_t ms0Addr,
		 	 	 	 	   uint32_t ms1Addr,
		 	 	 	 	   bool useMS0,
		 	 	 	 	   bool useMS1,
		 	 	 	 	   double waterDensity,
		 	 	 	 	   double atmosphericPressure) :
		 	 	 	 	   _ms0Addr(ms0Addr),
		 	 	 	 	   _ms1Addr(ms1Addr),
		 	 	 	 	   _useMS0(useMS0),
		 	 	 	 	   _useMS1(useMS1),
		 	 	 	 	   _waterDensity(waterDensity),
		 	 	 	 	   _atmosphericPressure(atmosphericPressure),
		 	 	 	 	   _conversionFactor((PA2MBARCONVERSION / (G * _waterDensity))) {
	initSensors();
	_isThreadRunning = false;
	reset();
	std::cout << "PSController initialized" << std::endl;
}

PSController::~PSController() {
	_thread->join();
	delete _ms0;
	delete _ms1;
}

void PSController::start() {
	_isThreadRunning = true;
	_thread = new std::thread(&PSController::run, this);
}

void PSController::reset() {
	if (_isThreadRunning) {
		std::lock_guard<std::mutex> lock(_mutex);
	}
	_avgPressure = 0;
	_depth = 0;
}

bool PSController::isThreadRunning() {
	return _isThreadRunning;
}

double PSController::getDepthInMeters() {
	std::lock_guard<std::mutex> lock(_mutex);
	return _depth;
}

MS5803* PSController::getMS0() {
	return _ms0;
}

MS5803* PSController::getMS1() {
	return _ms1;
}

void PSController::initSensors() {
	_ms0 = new MS5803(_ms0Addr);
	_ms1 = new MS5803(_ms1Addr);

	_ms0->resetSensor();
	_ms1->resetSensor();

	_ms0->initSensor();
	_ms1->initSensor();
}

void PSController::pollSensors() {
	_ms0->readSensor();
	_ms1->readSensor();

	std::lock_guard<std::mutex> lock(_mutex);
	_avgPressure = 0;
	if (_useMS0) {
		_avgPressure += _ms0->pressure();
	}
	if (_useMS1) {
		_avgPressure += _ms1->pressure();
	}
	if (_useMS0 && _useMS1) {
		_avgPressure *= (0.5);
	}
}

void PSController::calculateDepth() {
	std::lock_guard<std::mutex> lock(_mutex);
	_depth = (_avgPressure - _atmosphericPressure) * _conversionFactor;
}

void PSController::run() {
	std::cout << "PSController started" << std::endl;
	while(1) {
		pollSensors();
		calculateDepth();
		std::this_thread::sleep_for(std::chrono::milliseconds(PSLOOPTIME));
	}
}
