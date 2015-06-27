/*
 * PSController.h
 *
 *  Created on: Jun 26, 2015
 *      Author: Rahul
 */

#ifndef PSCONTROLLER_H_
#define PSCONTROLLER_H_

#include <thread>

#include "ms5803.h"

#define MS0ADDR 0x00020000
#define MS1ADDR 0x00030000

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

	std::thread *_thread;
};

#endif /* PSCONTROLLER_H_ */
