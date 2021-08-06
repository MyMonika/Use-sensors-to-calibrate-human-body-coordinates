#pragma once

#include <string>

using namespace zen;
class SIData
{
public:
	uintptr_t  handle;
	std::pair<ZenSensorInitError, ZenSensor>* sensorPair=new std::pair<ZenSensorInitError, ZenSensor>();
	ZenSensor* sensor = new ZenSensor();

	std::pair<bool, ZenSensorComponent> imuPair;
	ZenSensorComponent imu;

	// Get a string property
	std::pair<ZenError, std::string>sensorModelPair;
	std::string sensorModelName;


};


