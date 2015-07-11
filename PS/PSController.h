/*
 * PSController.h
 *
 *  Created on: Jun 26, 2015
 *      Author: Rahul
 */

#ifndef PSCONTROLLER_H_
#define PSCONTROLLER_H_

#include <iostream>
#include <thread>
#include <mutex>

#include "ms5803.h"
#include "../Utilities/Constants.h"

class PSController {
public:
	PSController(uint32_t ms0Addr,
				 uint32_t ms1Addr,
				 double waterDensity,
				 double atmosphericPressure);
	virtual ~PSController();

	void start();
	void reset();
	bool isThreadRunning();

	double getDepthInMeters();

	MS5803 *getMS0();
	MS5803 *getMS1();

private:
	void initSensors();
	void pollSensors();
	void calculateDepth();

	void run();

	const uint32_t _ms0Addr;
	const uint32_t _ms1Addr;
	MS5803 *_ms0;
	MS5803 *_ms1;

	double _avgPressure;
	double _depth;

	double _waterDensity;
	double _atmosphericPressure;
	const double _conversionFactor;

	std::thread *_thread;
	std::mutex _mutex;
	bool _isThreadRunning;
};

#endif /* PSCONTROLLER_H_ */
