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

class PWMController {
public:
	PWMController(NetworkClient *networkClient,
				  IMUController *imuController,
				  PSController *psController);
	virtual ~PWMController();

	void start();
	void stop();
};

#endif /* PWMCONTROLLER_H_ */
