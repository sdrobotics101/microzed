/*
 * PWMController.h
 *
 *  Created on: Jun 26, 2015
 *      Author: Rahul
 */

#ifndef PWMCONTROLLER_H_
#define PWMCONTROLLER_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "pwm.h"

#include "../Network/NetworkClient.hpp"
#include "../IMU/IMUController.h"
#include "../PS/PSController.h"
#include "../Utilities/PIDController.h"

#define PWMADDR 0x00010000

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

#define MXF1 0
#define MXF2 1
#define MXF3 2
#define MXF4 3
#define MXR1 4
#define MXR2 5
#define MXR3 6
#define MXR4 7
#define MYF1 8
#define MYF2 9
#define MYF3 10
#define MYF4 11
#define MYR1 12
#define MYR2 13
#define MYR3 14
#define MYR4 15
#define MZF1 16
#define MZF2 17
#define MZF3 18
#define MZF4 19
#define MZR1 20
#define MZR2 21
#define MZR3 22
#define MZR4 23

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

	PIDController _xRotationController;
	PIDController _yRotationController;
	PIDController _zRotationController;
	PIDController _depthController;

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

	Eigen::Vector3d _linearMotion;
	Eigen::Vector3d _rotationalMotion;

	double _pwmOutputs[24];
};

#endif /* PWMCONTROLLER_H_ */
