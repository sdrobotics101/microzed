/*
 * PIDController.h
 *
 *  Created on: Jul 2, 2015
 *      Author: Rahul
 */

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include <cmath>

#include "Timer.h"

class PIDController {
public:
	PIDController();
	PIDController(double p, double i, double d, double f);
	virtual ~PIDController();

	void start();
	void reset();

	void setPIDF(double p, double i, double d, double f);
	void setP(double p);
	void setI(double i);
	void setD(double d);
	void setF(double f);

	void setOutputLimits(double min, double max);
	void setInputLimits(double min, double max);
	void setContinuous(bool isContinuous);

	double getP();
	double getI();
	double getD();
	double getF();

	double getMinOutput();
	double getMaxOutput();
	double getMinInput();
	double getMaxInput();
	bool isContinuous();

	double getError();
	double getIntegral();
	double getDerivative();
	double getSetpoint();
	double getOutput();

	double calculateOutput(double input, double setpoint);
	double calculateOutput(double error);

private:
	double calculateOutput();

	double _p;
	double _i;
	double _d;
	double _f;

	double _minOutput;
	double _maxOutput;
	double _minInput;
	double _maxInput;
	bool _isContinuous;

	double _error;
	double _previousError;
	double _integral;
	double _derivative;
	double _setpoint;
	double _output;

	Timer _timer;
	double _dt;
	bool _isRunning;
};

#endif /* PIDCONTROLLER_H_ */
