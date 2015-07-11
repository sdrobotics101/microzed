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
		  	  	  	  	  	 double dF) :
		  	  	  	  	  	 _networkClient(networkClient),
		  	  	  	  	  	 _imuController(imuController),
		  	  	  	  	  	 _psController(psController),
		  	  	  	  	  	 _pwmAddr(pwmAddr),
		  	  	  	  	  	 _combinerRatio(combinerRatio) {
	_xRotationController.setPIDF(xP, xI, xD, xF);
	_xRotationController.setInputLimits(-180, 180);
	_xRotationController.setOutputLimits(PWMMINOUTPUT, PWMMAXOUTPUT);
	_xRotationController.setContinuous(true);

	_yRotationController.setPIDF(yP, yI, yD, yF);
	_yRotationController.setInputLimits(-180, 180);
	_yRotationController.setOutputLimits(PWMMINOUTPUT, PWMMAXOUTPUT);
	_yRotationController.setContinuous(true);

	_zRotationController.setPIDF(zP, zI, zD, zF);
	_zRotationController.setInputLimits(-180, 180);
	_zRotationController.setOutputLimits(PWMMINOUTPUT, PWMMAXOUTPUT);
	_zRotationController.setContinuous(true);

	_depthController.setPIDF(dP, dI, dD, dF);
	_depthController.setOutputLimits(PWMMINOUTPUT, PWMMAXOUTPUT);
	_depthController.setContinuous(false);

	for (int i = 0;i < NUMMOTORS;i++) {
		_pwmMap[i] = pwmMap[i];
	}

	for (int i = 0;i < NUMMOTORS;i++) {
		_pwmOutputs[i] = 0;
	}

	initPWM();
	std::cout << "PWMController initialized" << std::endl;
}

PWMController::~PWMController() {
	delete _pwm;
}

void PWMController::start() {
	_xRotationController.start();
	_yRotationController.start();
	_zRotationController.start();
	_depthController.start();
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
	_xAngle = _imuController->getXRotation();
	_yAngle = _imuController->getYRotation();
	_zAngle = _imuController->getZRotation();
	_depth = _psController->getDepthInMeters();
}

void PWMController::calculateOutputs() {
	_linearMotion(XAXIS) = _velX;
	_linearMotion(YAXIS) = _velY;
	_linearMotion(ZAXIS) = _depthController.calculateOutput(_depth, _posZ);

	Eigen::AngleAxisd xRotationMatrix(-_xAngle * (M_PI/180), Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd yRotationMatrix(-_yAngle * (M_PI/180), Eigen::Vector3d::UnitY());
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

void PWMController::writeOutputs() {
	_pwm->setDuty(_pwmOutputs);

	_networkClient->get_m2n_standard_packet()->set_orient_x(_xAngle);
	_networkClient->get_m2n_standard_packet()->set_orient_y(_yAngle);
	_networkClient->get_m2n_standard_packet()->set_orient_z(_zAngle);
	_networkClient->get_m2n_standard_packet()->set_pos_z(_depth);

	std::cout << "X: " << _xAngle << std::endl;
	std::cout << "Y: " << _yAngle << std::endl;
	std::cout << "Z: " << _zAngle << std::endl;
	std::cout << "D: " << _depth << std::endl;
}

double PWMController::combineMotion(double linear, double rotational1, double rotational2) {
	return ((_combinerRatio * linear) +
			(((1 - _combinerRatio) / 2) * rotational1) +
			(((1 - _combinerRatio) / 2) * rotational2));
}

double PWMController::linearize(int motor, double speed) {

    /* speed calibration based on slowest motor */
    double motor_cal[24] = {0.9861, 0.9301, 0.9816, 0.9682,
                            0.9064, 0.9301, 1.0000, 0.8950,
                            0.9425, 0.9025, 0.9638, 0.9816,
                            0.9301, 0.9261, 0.9025, 0.9816,
                            0.9181, 0.9261, 0.8068, 0.9142,
                            0.9301, 0.9342, 0.9861, 0.9425};

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
	std::cout << "PWMController started" << std::endl;
	while(1) {
		pollData();
		calculateOutputs();
		writeOutputs();
		std::this_thread::sleep_for(std::chrono::milliseconds(PWMLOOPTIME));
	}
}
