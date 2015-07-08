/*
 * Cubeception.cpp
 *
 *  Created on: Jul 7, 2015
 *      Author: Rahul
 */

#include "Cubeception.h"

Cubeception::Cubeception(std::string configFile) {
	_iniReader = new INIReader(configFile);
	_networkClient = new NetworkClient();
	_imuController = new IMUController();
	_psController = new PSController();
	_pwmController = new PWMController();
}

Cubeception::~Cubeception() {
	delete _iniReader;
	delete _networkClient;
	delete _imuController;
	delete _psController;
	delete _pwmController;
}

void Cubeception::start() {
	_networkClient->start();
	_imuController->start();
	_psController->start();
	_pwmController->start();
}
