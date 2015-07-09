/*
 * PSController.cpp
 *
 *  Created on: Jun 26, 2015
 *      Author: Rahul
 */

#include "PSController.h"

PSController::PSController(uint32_t ms0Addr,
		 	 	 	 	   uint32_t ms1Addr,
		 	 	 	 	   double waterDensity,
		 	 	 	 	   double atmosphericPressure) :
		 	 	 	 	   _ms0Addr(ms0Addr),
		 	 	 	 	   _ms1Addr(ms1Addr),
		 	 	 	 	   _waterDensity(waterDensity),
		 	 	 	 	   _atmosphericPressure(atmosphericPressure),
		 	 	 	 	   _conversionFactor((PA2MBARCONVERSION / (G * _waterDensity))) {
	initSensors();
	_avgPressure = 0;
	_depth = 0;
}

PSController::~PSController() {
	_thread->join();
	delete _ms0;
	delete _ms1;
}

void PSController::start() {
	_thread = new std::thread(&PSController::run, this);
}

void PSController::reset() {
	std::lock_guard<std::mutex> lock(_mutex);
	_avgPressure = 0;
	_depth = 0;
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
	_avgPressure = ((_ms0->pressure() + _ms1->pressure()) / 2);
}

void PSController::calculateDepth() {
	std::lock_guard<std::mutex> lock(_mutex);
	_depth = (_avgPressure - _atmosphericPressure) * _conversionFactor;
}

void PSController::run() {
	while(1) {
		pollSensors();
		calculateDepth();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}
