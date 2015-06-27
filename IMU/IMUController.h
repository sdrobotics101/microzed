/*
 * IMUController.h
 *
 *  Created on: Jun 26, 2015
 *      Author: Rahul
 */

#ifndef IMUCONTROLLER_H_
#define IMUCONTROLLER_H_

#include <thread>

#include "mpu9250.h"

#define MPU0ADDR 0x00040000
#define MPU1ADDR 0x00050000

class IMUController {
public:
	IMUController();
	virtual ~IMUController();

	void start();
	void reset();

	double getXRotation();
	double getYRotation();
	double getZRotation();

	MPU9250 *getMPU0();
	MPU9250 *getMPU1();

private:
	void initSensors();
	void pollSensors();
	void calculateOrientation();

	void run();

	MPU9250 *_mpu0;
	MPU9250 *_mpu1;

	double _avgAX;
	double _avgAY;
	double _avgAZ;
	double _avgGX;
	double _avgGY;
	double _avgGZ;
	double _avgMX;
	double _avgMY;
	double _avgMZ;

	double _xAngle;
	double _yAngle;
	double _zAngle;

	std::thread *_thread;
};

#endif /* IMUCONTROLLER_H_ */
