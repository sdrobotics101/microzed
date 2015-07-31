/*
 * PWMController.cpp
 *
 *  Created on: Jun 26, 2015
 *      Author: Rahul
 */

#include "PWMController.h"

PWMController::PWMController(NetworkClient *networkClient,
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
		  	  	  	  	  	 double zIntegratorLimit) :
		  	  	  	  	  	 _networkClient(networkClient),
		  	  	  	  	  	 _imuController(imuController),
		  	  	  	  	  	 _psController(psController),
		  	  	  	  	  	 _pwmAddr(pwmAddr),
		  	  	  	  	  	 _killRegister(killRegister),
		  	  	  	  	  	 _killBit(killBit),
		  	  	  	  	  	 _killThreshold(killThreshold),
		  	  	  	  	  	 _killState(killState),
		  	  	  	  	  	 _combinerRatio(combinerRatio) {
	_xRotationController.setPIDF(xP, xI, xD, xF);
	_xRotationController.setInputLimits(-180, 180);
	_xRotationController.setOutputLimits(PWMMINOUTPUT, PWMMAXOUTPUT);
	_xRotationController.setContinuous(true);
	_xRotationController.setTolerance(xTolerance);
	_xRotationController.setIntegratorLimit(xIntegratorLimit);

	_yRotationController.setPIDF(yP, yI, yD, yF);
	_yRotationController.setInputLimits(-180, 180);
	_yRotationController.setOutputLimits(PWMMINOUTPUT, PWMMAXOUTPUT);
	_yRotationController.setContinuous(true);
	_yRotationController.setTolerance(yTolerance);
	_yRotationController.setIntegratorLimit(yIntegratorLimit);

	_zRotationController.setPIDF(zP, zI, zD, zF);
	_zRotationController.setInputLimits(-180, 180);
	_zRotationController.setOutputLimits(PWMMINOUTPUT, PWMMAXOUTPUT);
	_zRotationController.setContinuous(true);
	_zRotationController.setTolerance(zTolerance);
	_zRotationController.setIntegratorLimit(zIntegratorLimit);

	_depthController.setPIDF(dP, dI, dD, dF);
	_depthController.setOutputLimits(PWMMINOUTPUT, PWMMAXOUTPUT);
	_depthController.setContinuous(false);
	_depthController.setTolerance(depthTolerance);

	for (int i = 0;i < NUMMOTORS;i++) {
		_pwmMap[i] = pwmMap[i];
	}

	for (int i = 0;i < NUMMOTORS;i++) {
		_pwmOutputs[i] = 0;
	}

	_killCount = 0;

	_sensorMode = _imuController->getSensorMode();

	initPWM();
	//std::cout << "PWMController initialized" << std::endl;
}

PWMController::~PWMController() {
	delete _pwm;
}

void PWMController::start() {
	_xRotationController.start();
	_yRotationController.start();
	_zRotationController.start();
	_depthController.start();
	_timer.start();
	run();
}

void PWMController::stop() {
	for (int i = 0;i < NUMMOTORS;i++) {
		_pwmOutputs[i] = 0;
	}
	_pwm->setDuty(_pwmOutputs);
	_pwm->disable();
}

void PWMController::initPWM() {
	_pwm = new PWM(_pwmAddr);
	_pwm->setMap(_pwmMap);
	_pwm->enable();
	_pwm->setDuty(_pwmOutputs);
}

void PWMController::pollData() {
	_velX = _networkClient->get_n2m_standard_packet()->get_vel_x();
	_velY = _networkClient->get_n2m_standard_packet()->get_vel_y();
	_posZ = _networkClient->get_n2m_standard_packet()->get_pos_z();
	_rotX = _networkClient->get_n2m_standard_packet()->get_rot_x();
	_rotY = _networkClient->get_n2m_standard_packet()->get_rot_y();
	_rotZ = _networkClient->get_n2m_standard_packet()->get_rot_z();

	if (_networkClient->get_n2m_standard_packet()->get_mode()[0]) {
		_imuController->setSensorsToUse(SENSORMODEGYROONLY);
	} else if (_networkClient->get_n2m_standard_packet()->get_mode()[1]) {
		_imuController->setSensorsToUse(SENSORMODEACCMAGONLY);
	} else {
		_imuController->setSensorsToUse(SENSORMODEBOTH);
	}

	if (_imuController->getSensorMode() != _sensorMode) {
		_imuController->reset();
		_sensorMode = _imuController->getSensorMode();
	}

	_xAngle = _imuController->getXRotation();
	_yAngle = _imuController->getYRotation();
	_zAngle = _imuController->getZRotation();
	_depth = _psController->getDepthInMeters();
}

void PWMController::calculateOutputs() {
	_linearMotion(XAXIS) = _velX;
	_linearMotion(YAXIS) = _velY;
	_linearMotion(ZAXIS) = _depthController.calculateOutput(_depth, _posZ);

	Eigen::AngleAxisd xRotationMatrix(0/*-_xAngle * (M_PI/180)*/, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd yRotationMatrix(0/*-_yAngle * (M_PI/180)*/, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd zRotationMatrix(-_zAngle * (M_PI/180), Eigen::Vector3d::UnitZ());

	_linearMotion = xRotationMatrix.toRotationMatrix() *
					yRotationMatrix.toRotationMatrix() *
					zRotationMatrix.toRotationMatrix() *
					_linearMotion;

	_rotationalMotion(XAXIS) = _xRotationController.calculateOutput(_xAngle, _rotX);
	_rotationalMotion(YAXIS) = _yRotationController.calculateOutput(_yAngle, _rotY);
	_rotationalMotion(ZAXIS) = _zRotationController.calculateOutput(_zAngle, _rotZ);

	_pwmOutputs[MXF1] = combineMotion(_linearMotion(XAXIS),
									  -_rotationalMotion(YAXIS),
									  _rotationalMotion(ZAXIS));
	_pwmOutputs[MXF2] = combineMotion(_linearMotion(XAXIS),
									  _rotationalMotion(YAXIS),
									  _rotationalMotion(ZAXIS));
	_pwmOutputs[MXF3] = combineMotion(_linearMotion(XAXIS),
			  	  	  	  	  	  	  _rotationalMotion(YAXIS),
			  	  	  	  	  	  	  -_rotationalMotion(ZAXIS));
	_pwmOutputs[MXF4] = combineMotion(_linearMotion(XAXIS),
			  	  	  	  	  	  	  -_rotationalMotion(YAXIS),
			  	  	  	  	  	  	  -_rotationalMotion(ZAXIS));

	_pwmOutputs[MXR1] = combineMotion(-_linearMotion(XAXIS),
			  	  	  	  	  	  	  _rotationalMotion(YAXIS),
			  	  	  	  	  	  	  -_rotationalMotion(ZAXIS));
	_pwmOutputs[MXR2] = combineMotion(-_linearMotion(XAXIS),
			  	  	  	  	  	  	  -_rotationalMotion(YAXIS),
			  	  	  	  	  	  	  -_rotationalMotion(ZAXIS));
	_pwmOutputs[MXR3] = combineMotion(-_linearMotion(XAXIS),
			  	  	  	  	  	  	  -_rotationalMotion(YAXIS),
			  	  	  	  	  	  	  _rotationalMotion(ZAXIS));
	_pwmOutputs[MXR4] = combineMotion(-_linearMotion(XAXIS),
			  	  	  	  	  	  	  _rotationalMotion(YAXIS),
			  	  	  	  	  	  	  _rotationalMotion(ZAXIS));

	_pwmOutputs[MYF1] = combineMotion(_linearMotion(YAXIS),
									  _rotationalMotion(XAXIS),
									  _rotationalMotion(ZAXIS));
	_pwmOutputs[MYF2] = combineMotion(_linearMotion(YAXIS),
			  	  	  	  	  	  	  -_rotationalMotion(XAXIS),
			  	  	  	  	  	  	  _rotationalMotion(ZAXIS));
	_pwmOutputs[MYF3] = combineMotion(_linearMotion(YAXIS),
			  	  	  	  	  	  	  -_rotationalMotion(XAXIS),
			  	  	  	  	  	  	  -_rotationalMotion(ZAXIS));
	_pwmOutputs[MYF4] = combineMotion(_linearMotion(YAXIS),
			  	  	  	  	  	  	  _rotationalMotion(XAXIS),
			  	  	  	  	  	  	  -_rotationalMotion(ZAXIS));

	_pwmOutputs[MYR1] = combineMotion(-_linearMotion(YAXIS),
									  -_rotationalMotion(XAXIS),
									  -_rotationalMotion(ZAXIS));
	_pwmOutputs[MYR2] = combineMotion(-_linearMotion(YAXIS),
			  	  	  	  	  	  	  _rotationalMotion(XAXIS),
			  	  	  	  	  	  	  -_rotationalMotion(ZAXIS));
	_pwmOutputs[MYR3] = combineMotion(-_linearMotion(YAXIS),
			  	  	  	  	  	  	  _rotationalMotion(XAXIS),
			  	  	  	  	  	  	  _rotationalMotion(ZAXIS));
	_pwmOutputs[MYR4] = combineMotion(-_linearMotion(YAXIS),
			  	  	  	  	  	  	  -_rotationalMotion(XAXIS),
			  	  	  	  	  	  	  _rotationalMotion(ZAXIS));

	_pwmOutputs[MZF1] = combineMotion(_linearMotion(ZAXIS),
									  -_rotationalMotion(XAXIS),
									  -_rotationalMotion(YAXIS));
	_pwmOutputs[MZF2] = combineMotion(_linearMotion(ZAXIS),
									  -_rotationalMotion(XAXIS),
									  _rotationalMotion(YAXIS));
	_pwmOutputs[MZF3] = combineMotion(_linearMotion(ZAXIS),
			  	  	  	  	  	  	  _rotationalMotion(XAXIS),
			  	  	  	  	  	  	  _rotationalMotion(YAXIS));
	_pwmOutputs[MZF4] = combineMotion(_linearMotion(ZAXIS),
			  	  	  	  	  	  	  _rotationalMotion(XAXIS),
			  	  	  	  	  	  	  -_rotationalMotion(YAXIS));

	_pwmOutputs[MZR1] = combineMotion(-_linearMotion(ZAXIS),
									  _rotationalMotion(XAXIS),
									  _rotationalMotion(YAXIS));
	_pwmOutputs[MZR2] = combineMotion(-_linearMotion(ZAXIS),
									  _rotationalMotion(XAXIS),
									  -_rotationalMotion(YAXIS));
	_pwmOutputs[MZR3] = combineMotion(-_linearMotion(ZAXIS),
			  	  	  	  	  	  	  -_rotationalMotion(XAXIS),
			  	  	  	  	  	  	  -_rotationalMotion(YAXIS));
	_pwmOutputs[MZR4] = combineMotion(-_linearMotion(ZAXIS),
			  	  	  	  	  	  	  -_rotationalMotion(XAXIS),
			  	  	  	  	  	  	  _rotationalMotion(YAXIS));

	for (int i = 0;i < NUMMOTORS;i++) {
		if (_pwmOutputs[i] < 0) {
			_pwmOutputs[i] = 0;
		}
	}

	for (int i = 0;i < NUMMOTORS;i++) {
		_pwmOutputs[i] = linearize(i, _pwmOutputs[i]);
	}
}

void PWMController::writeOutputs(bool isKilled) {
	_pwm->setDuty(_pwmOutputs);

	_networkClient->get_m2n_standard_packet()->set_orient_x(_xAngle);
	_networkClient->get_m2n_standard_packet()->set_orient_y(_yAngle);
	_networkClient->get_m2n_standard_packet()->set_orient_z(_zAngle);
	_networkClient->get_m2n_standard_packet()->set_pos_z(_depth);

	if (isKilled) {
		_networkClient->get_m2n_standard_packet()->set_health(0.0);
	} else {
		_networkClient->get_m2n_standard_packet()->set_health(1.0);
	}

	for (int i = 0;i < 23;i++) {
		std::cout << _pwmOutputs[i] << ",";
	}
	std::cout << _pwmOutputs[23] << std::endl;

	printf("%8.4f,%8.4f,%8.4f,%8.4f,%8.4f\n", _xAngle, _yAngle, _zAngle, _depth, _timer.dt());
	std::cout.flush();
}

bool PWMController::isKilled() {
	uint32_t result = 0;
	regio_rd32(_killRegister, &result, 0);
	if (((result & _killBit) > 0) && _killState) {
		_killCount++;
	} else {
		_killCount = 0;
	}

	if (_killCount > _killThreshold) {
		return true;
	} else {
		return false;
	}
}

double PWMController::combineMotion(double linear, double rotational1, double rotational2) {
	return ((_combinerRatio * linear) +
			(((1 - _combinerRatio) / 2) * rotational1) +
			(((1 - _combinerRatio) / 2) * rotational2));
}

double PWMController::linearize(int motor, double speed) {

    /* speed calibration based on slowest motor */
    double motor_cal[24] = {0.8182, 0.8674, 0.8220, 0.8333,
            				0.8902, 0.8674, 0.9205, 0.9015,
            				0.8144, 0.8939, 0.8371, 0.8220,
            				0.8674, 0.8712, 0.8939, 0.8220,
            				0.8788, 0.8712, 1.0000, 0.8826,
            				0.8674, 0.8636, 0.8182, 0.8561};

    /* polynomial coefficients for linearizer curve fit */
    double poly[3] = {0.003666, -0.659065, 29.217563};

    /* figure out the adjustment based on 2nd order polynomial interpolation */
    double adj;
    if (speed < 10.0) {
        adj = poly[0]*pow(speed, 2.0) + poly[1]*speed + 0.0;
    }
    else if (speed < 20.0) {
        adj = poly[0]*pow(speed, 2.0) + poly[1]*speed + 0.9*poly[2];
    }
    else {
        adj = poly[0]*pow(speed, 2.0) + poly[1]*speed + poly[2];
    }

    adj += speed;                   /* adjust the current speed */
    adj *= motor_cal[motor];        /* derate the speed based on slowest motor */

    /* saturate outside {0, 100} interval */
    if      (adj <   0.0) { adj =   0.0; }
    else if (adj > 100.0) { adj = 100.0; }

    return (adj);
}


void PWMController::run() {
	//std::cout << "PWMController started" << std::endl;
	while(1) {
	//	pollData();
	//	calculateOutputs();
	//	writeOutputs();
	//	std::this_thread::sleep_for(std::chrono::milliseconds(PWMLOOPTIME));
	//}

		while (!isKilled()) {
			pollData();
			calculateOutputs();
			writeOutputs(false);
			std::this_thread::sleep_for(std::chrono::milliseconds(PWMLOOPTIME));
		}
		stop();
		std::cout << "KILLED" << std::endl;

		while (isKilled()) {
			pollData();
			writeOutputs(true);
			std::this_thread::sleep_for(std::chrono::milliseconds(PWMLOOPTIME));
		}
		_imuController->reset();

		_xRotationController.reset();
		_yRotationController.reset();
		_zRotationController.reset();
		_depthController.reset();

		_xRotationController.start();
		_yRotationController.start();
		_zRotationController.start();
		_depthController.start();

		_pwm->enable();
		std::cout << "ACTIVE" << std::endl;
	}
}
