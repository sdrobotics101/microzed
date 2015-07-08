/*
 * Cubeception.h
 *
 *  Created on: Jul 7, 2015
 *      Author: Rahul
 */

#ifndef CUBECEPTION_H_
#define CUBECEPTION_H_

#include "IMU/IMUController.h"
#include "PS/PSController.h"
#include "PWM/PWMController.h"
#include "Network/NetworkClient.hpp"
#include "Utilities/INI/INIReader.h"

class Cubeception {
public:
	Cubeception(std::string configFile);
	virtual ~Cubeception();

	void start();

private:
	INIReader *_iniReader;
	NetworkClient *_networkClient;
	IMUController *_imuController;
	PSController *_psController;
	PWMController *_pwmController;
};

#endif /* CUBECEPTION_H_ */
