/*
 * PWMController.cpp
 *
 *  Created on: Jun 26, 2015
 *      Author: Rahul
 */

#include "PWMController.h"

PWMController::PWMController(NetworkClient *networkClient,
		  	  	  	  	  	 IMUController *imuController,
		  	  	  	  	  	 PSController *psController) :
		  	  	  	  	  	 _networkClient(networkClient),
		  	  	  	  	  	 _imuController(imuController),
		  	  	  	  	  	 _psController(psController) {
	_networkClient->open("192.168.1.21", 8888);

	_xRotationController.setPIDF(0, 0, 0, 0);
	_xRotationController.setInputLimits(-180, 180);
	_xRotationController.setOutputLimits(-100, 100);
	_xRotationController.setContinuous(true);

	_yRotationController.setPIDF(0, 0, 0, 0);
	_yRotationController.setInputLimits(-180, 180);
	_yRotationController.setOutputLimits(-100, 100);
	_yRotationController.setContinuous(true);

	_zRotationController.setPIDF(0, 0, 0, 0);
	_zRotationController.setInputLimits(-180, 180);
	_zRotationController.setOutputLimits(-100, 100);
	_zRotationController.setContinuous(true);

	_depthController.setPIDF(0, 0, 0, 0);
	_depthController.setOutputLimits(-100, 100);
	_depthController.setContinuous(false);

	initPWM();
}

PWMController::~PWMController() {
	delete _pwm;
}

void PWMController::start() {
	_networkClient->start();
	_imuController->start();
	_psController->start();
	_xRotationController.start();
	_yRotationController.start();
	_zRotationController.start();
	_depthController.start();
	run();
}

void PWMController::stop() {
	_pwm->disable();
}

void PWMController::initPWM() {
	_pwm = new PWM(PWMADDR);
	uint32_t map[24] = {0, 1, 2, 3,
						4, 5, 6, 7,
						8, 9, 10, 11,
						12, 13, 14, 15,
						16, 17, 18, 19,
						20, 21, 22, 23};
	_pwm->setMap(map);
	_pwm->enable();
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

	Eigen::AngleAxisd xRotationMatrix(_xAngle, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd yRotationMatrix(_yAngle, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd zRotationMatrix(_zAngle, Eigen::Vector3d::UnitZ());

	_linearMotion = xRotationMatrix.toRotationMatrix() *
					yRotationMatrix.toRotationMatrix() *
					zRotationMatrix.toRotationMatrix() *
					_linearMotion;

	_rotationalMotion(XAXIS) = _xRotationController.calculateOutput(_xAngle, _rotX);
	_rotationalMotion(YAXIS) = _yRotationController.calculateOutput(_yAngle, _rotY);
	_rotationalMotion(ZAXIS) = _zRotationController.calculateOutput(_zAngle, _rotZ);

	_pwmOutputs[MXF1] = _linearMotion(XAXIS);
	_pwmOutputs[MXF2] = _linearMotion(XAXIS);
	_pwmOutputs[MXF3] = _linearMotion(XAXIS);
	_pwmOutputs[MXF4] = _linearMotion(XAXIS);

	_pwmOutputs[MXR1] = -_linearMotion(XAXIS);
	_pwmOutputs[MXR2] = -_linearMotion(XAXIS);
	_pwmOutputs[MXR3] = -_linearMotion(XAXIS);
	_pwmOutputs[MXR4] = -_linearMotion(XAXIS);

	_pwmOutputs[MYF1] = _linearMotion(YAXIS);
	_pwmOutputs[MYF2] = _linearMotion(YAXIS);
	_pwmOutputs[MYF3] = _linearMotion(YAXIS);
	_pwmOutputs[MYF4] = _linearMotion(YAXIS);

	_pwmOutputs[MYR1] = -_linearMotion(YAXIS);
	_pwmOutputs[MYR2] = -_linearMotion(YAXIS);
	_pwmOutputs[MYR3] = -_linearMotion(YAXIS);
	_pwmOutputs[MYR4] = -_linearMotion(YAXIS);

	_pwmOutputs[MZF1] = _linearMotion(ZAXIS);
	_pwmOutputs[MZF2] = _linearMotion(ZAXIS);
	_pwmOutputs[MZF3] = _linearMotion(ZAXIS);
	_pwmOutputs[MZF4] = _linearMotion(ZAXIS);

	_pwmOutputs[MZR1] = -_linearMotion(ZAXIS);
	_pwmOutputs[MZR2] = -_linearMotion(ZAXIS);
	_pwmOutputs[MZR3] = -_linearMotion(ZAXIS);
	_pwmOutputs[MZR4] = -_linearMotion(ZAXIS);

	for(int i = 0;i < 24;i++) {
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

void PWMController::run() {
	while(1) {
		pollData();
		calculateOutputs();
		writeOutputs();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}
