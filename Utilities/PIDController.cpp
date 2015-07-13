/*
 * PIDController.cpp
 *
 *  Created on: Jul 2, 2015
 *      Author: Rahul
 */

#include "PIDController.h"

PIDController::PIDController() : PIDController(0, 0, 0, 0) {
}

PIDController::PIDController(double p, double i, double d, double f) :
							_p(p),
							_i(i),
							_d(d),
							_f(f) {
	reset();
	_minOutput = 0;
	_maxOutput = 0;
	_minInput = 0;
	_minOutput = 0;
	_isContinuous = false;
	_tolerance = 0;
	_isWithinTolerance = false;
	_integratorLimit = 0;
	_isRunning = false;
}

PIDController::~PIDController() {

}

void PIDController::start() {
	_timer.start();
	_isRunning = true;
}

void PIDController::reset() {
	_error = 0;
	_previousError = 0;
	_integral = 0;
	_derivative = 0;
	_setpoint = 0;
	_output = 0;
}

void PIDController::setPIDF(double p, double i, double d, double f) {
	_p = p;
	_i = i;
	_d = d;
	_f = f;
}

void PIDController::setP(double p) {
	_p = p;
}

void PIDController::setI(double i) {
	_i = i;
}

void PIDController::setD(double d) {
	_d = d;
}

void PIDController::setF(double f) {
	_f = f;
}

void PIDController::setOutputLimits(double min, double max) {
	_minOutput = min;
	_maxOutput = max;
}

void PIDController::setInputLimits(double min, double max) {
	_minInput = min;
	_maxInput = max;
}

void PIDController::setContinuous(bool isContinuous) {
	_isContinuous = isContinuous;
}

void PIDController::setTolerance(double tolerance) {
	_tolerance = tolerance;
}

void PIDController::setIntegratorLimit(double limit) {
	_integratorLimit = limit;
}

double PIDController::getP() {
	return _p;
}

double PIDController::getI() {
	return _i;
}

double PIDController::getD() {
	return _d;
}

double PIDController::getF() {
	return _f;
}

double PIDController::getMinOutput() {
	return _minOutput;
}

double PIDController::getMaxOutput() {
	return _maxOutput;
}

double PIDController::getMinInput() {
	return _minInput;
}

double PIDController::getMaxInput() {
	return _maxInput;
}

bool PIDController::isContinuous() {
	return _isContinuous;
}

bool PIDController::isWithinTolerance() {
	return _isWithinTolerance;
}

double PIDController::getError() {
	return _error;
}

double PIDController::getIntegral() {
	return _integral;
}

double PIDController::getDerivative() {
	return _derivative;
}

double PIDController::getSetpoint() {
	return _setpoint;
}

double PIDController::getOutput() {
	return _output;
}

double PIDController::calculateOutput(double input, double setpoint) {
	if ((_minInput != 0) && (_maxInput != 0)) {
		if (input > _maxInput) {
			input = _maxInput;
		} else if (input < _minInput) {
			input = _minInput;
		}
	}

	_error = setpoint - input;
	_setpoint = setpoint;

	if (input > _setpoint - _tolerance && input < _setpoint + _tolerance) {
		_isWithinTolerance = true;
	} else {
		_isWithinTolerance = false;
	}

	return calculateOutput();
}

double PIDController::calculateOutput(double error) {
	_error = error;
	_setpoint = 0;
	return calculateOutput();
}

double PIDController::calculateOutput() {
	if (_isContinuous) {
		if (std::abs(_error) > ((std::abs(_minInput) + std::abs(_maxInput)) / 2)) {
			if (_error > 0) {
				_error = _error - _maxInput + _minInput;
			} else {
				_error = _error + _maxInput - _minInput;
			}
		}
	}

	_dt = _timer.dt();
	_integral += _error * _dt;
	_derivative = ((_error - _previousError) / _dt);
	_previousError = _error;

	if (_integratorLimit != 0 && std::abs(_error) > _integratorLimit) {
		_integral = 0;
	}

	_output = (_p * _error) + (_i * _integral) + (_d * _derivative) + (_f * _setpoint);

	if ((_minOutput != 0) && (_maxOutput != 0)) {
		if (_output > _maxOutput) {
			_output = _maxOutput;
		} else if (_output < _minOutput) {
			_output = _minOutput;
		}
	}

	return _output;
}





















