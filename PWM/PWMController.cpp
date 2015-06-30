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
	_networkClient->start();
	_imuController->start();
	_psController->start();
	initPWM();
}

PWMController::~PWMController() {
	delete _pwm;
}

void PWMController::start() {
	run();
}

void PWMController::stop() {
	_pwm->disable();
}

void PWMController::initPWM() {
	_pwm = new PWM(PWMADDR);
	//pwm->setMap();
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
