/*
 * PWMController.h
 *
 *  Created on: Jun 26, 2015
 *      Author: Rahul
 */

#ifndef PWMCONTROLLER_H_
#define PWMCONTROLLER_H_

#include "pwm.h"

#include "../Network/NetworkClient.hpp"
#include "../IMU/IMUController.h"
#include "../PS/PSController.h"

#define PWMADDR 0x00010000

class PWMController {
public:
	PWMController(NetworkClient *networkClient,
				  IMUController *imuController,
				  PSController *psController);
	virtual ~PWMController();

	void start();
	void stop();

private:
	void initPWM();
	void pollData();
	void calculateOutputs();
	void writeOutputs();

	void run();

	NetworkClient *_networkClient;
	IMUController *_imuController;
	PSController *_psController;

	PWM *_pwm;

	double _velX;
	double _velY;
	double _posZ;
	double _rotX;
	double _rotY;
	double _rotZ;
	double _xAngle;
	double _yAngle;
	double _zAngle;
	double _depth;

	double _pwmOutputs[24];
};

#endif /* PWMCONTROLLER_H_ */
