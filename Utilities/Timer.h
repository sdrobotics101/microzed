/*
 * Timer.h
 *
 *  Created on: Jul 2, 2015
 *      Author: Rahul
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>

#include "regio.h"

#define CLOCKADDR 0x00000018

class Timer {
public:
	Timer();
	virtual ~Timer();

	void start();
	void reset();

	double dt();

private:
	uint32_t _lastCount;
	uint32_t _currentCount;
	double _dt;
};

#endif /* TIMER_H_ */
