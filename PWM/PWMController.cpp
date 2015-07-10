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
	_xAngle = 0;//_imuController->getXRotation();
	_yAngle = 0;//_imuController->getYRotation();
	_zAngle = 0;//_imuController->getZRotation();
	_depth = 0;//_psController->getDepthInMeters();
}

void PWMController::calculateOutputs() {
	_linearMotion(XAXIS) = _velX;
	_linearMotion(YAXIS) = _velY;
	_linearMotion(ZAXIS) = _depthController.calculateOutput(_depth, _posZ);

	Eigen::AngleAxisd xRotationMatrix(-_xAngle, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd yRotationMatrix(-_yAngle, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd zRotationMatrix(-_zAngle, Eigen::Vector3d::UnitZ());

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
}

void PWMController::writeOutputs() {
	_pwm->setDuty(_pwmOutputs);

	_networkClient->get_m2n_standard_packet()->set_orient_x(_xAngle);
	_networkClient->get_m2n_standard_packet()->set_orient_y(_yAngle);
	_networkClient->get_m2n_standard_packet()->set_orient_z(_zAngle);
	_networkClient->get_m2n_standard_packet()->set_pos_z(_depth);
}

double PWMController::combineMotion(double linear, double rotational1, double rotational2) {
	return ((_combinerRatio * linear) +
			(((1 - _combinerRatio) / 2) * rotational1) +
			(((1 - _combinerRatio) / 2) * rotational2));
}

void PWMController::run() {
	while(1) {
		pollData();
		calculateOutputs();
		writeOutputs();
		std::this_thread::sleep_for(std::chrono::milliseconds(PWMLOOPTIME));
	}
}
