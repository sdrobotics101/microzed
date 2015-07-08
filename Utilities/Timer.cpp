/*
 * Timer.cpp
 *
 *  Created on: Jul 2, 2015
 *      Author: Rahul
 */

#include "Timer.h"

Timer::Timer() {
	reset();
}

Timer::~Timer() {

}

void Timer::start() {
	regio_rd32(CLOCKADDR, &_lastCount, 0);
}

void Timer::reset() {
	_lastCount = 0;
	_currentCount = 0;
}

double Timer::dt() {
	regio_rd32(CLOCKADDR, &_currentCount, 0);
	if (_currentCount < _lastCount) {
		_dt = (((_currentCount + UINT32_MAX) - _lastCount) / 100000000.0);
	} else {
		_dt = ((_currentCount - _lastCount) / 100000000.0);
	}
	_lastCount = _currentCount;
	return _dt;
}
