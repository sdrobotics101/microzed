/*
 * PSController.h
 *
 *  Created on: Jun 26, 2015
 *      Author: Rahul
 */

#ifndef PSCONTROLLER_H_
#define PSCONTROLLER_H_

#include <thread>
#include <mutex>

#include "ms5803.h"

#define MS0ADDR 0x00020000
#define MS1ADDR 0x00030000

#define WATERDENSITY 1000 //in kg/m^3
#define G 			 9.81 //in m/s^2
#define CONVERSION   100  //conversion factor from Pa to mBar
#define ATMOSPHERE	 1012.414 //in mBar

class PSController {
public:
	PSController();
	virtual ~PSController();

	void start();
	void reset();

	double getDepthInMeters();

	MS5803 *getMS0();
	MS5803 *getMS1();

private:
	void initSensors();
	void pollSensors();
	void calculateDepth();

	void run();

	MS5803 *_ms0;
	MS5803 *_ms1;

	double _avgPressure;
	double _depth;

	const double _conversionFactor;

	std::thread *_thread;
	std::mutex _mutex;
};

#endif /* PSCONTROLLER_H_ */
