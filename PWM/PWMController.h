/*
 * PWMController.h
 *
 *  Created on: Jun 26, 2015
 *      Author: Rahul
 */

#ifndef PWMCONTROLLER_H_
#define PWMCONTROLLER_H_

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "pwm.h"

#include "../Network/NetworkClient.hpp"
#include "../IMU/IMUController.h"
#include "../PS/PSController.h"
#include "../Utilities/PIDController.h"
#include "../Utilities/Constants.h"
#include "../Utilities/Timer.h"
#include "../Utilities/regio.h"

class PWMController {
public:
	PWMController(NetworkClient *networkClient,
				  IMUController *imuController,
				  PSController *psController,
				  uint32_t pwmAddr,
				  uint32_t pwmMap[NUMMOTORS],
				  uint32_t killRegister,
				  uint32_t killBit,
				  int killThreshold,
				  bool killState,
				  double combinerRatio,
				  double xP,
				  double xI,
				  double xD,
				  double xF,
				  double yP,
				  double yI,
				  double yD,
				  double yF,
				  double zP,
				  double zI,
				  double zD,
				  double zF,
				  double dP,
				  double dI,
				  double dD,
				  double dF,
				  double xTolerance,
				  double yTolerance,
				  double zTolerance,
				  double depthTolerance,
				  double xIntegratorLimit,
				  double yIntegratorLimit,
				  double zIntegratorLimit);
	virtual ~PWMController();

	void start();
	void stop();

private:
	void initPWM();
	void pollData();
	void calculateOutputs();
	void writeOutputs(bool isKilled);

	bool isKilled();

	double combineMotion(double linear, double rotational1, double rotational2);
	double linearize(int motor, double speed);

	void run();

	NetworkClient *_networkClient;
	IMUController *_imuController;
	PSController *_psController;

	PIDController _xRotationController;
	PIDController _yRotationController;
	PIDController _zRotationController;
	PIDController _depthController;

	Timer _timer;

	PWM *_pwm;
	const uint32_t _pwmAddr;
	uint32_t _pwmMap[NUMMOTORS];

	uint32_t _killRegister;
	uint32_t _killBit;
	int _killCount;
	int _killThreshold;
	bool _killState;

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

	int _sensorMode;

	Eigen::Vector3d _linearMotion;
	Eigen::Vector3d _rotationalMotion;
	const double _combinerRatio;

	double _pwmOutputs[NUMMOTORS];
};

#endif /* PWMCONTROLLER_H_ */
