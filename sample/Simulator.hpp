#pragma once
#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP
/*My includes*/
//#include "prettyprint.hpp"
#include <iostream>
#include "string.h"
extern "C"{
	#include "extApi.h"
	#include "v_repLib.h"
}
class Simulator{
private:
	int id;
	int portNumber;
	std::string ip;
public:
	const int ret_ok			=0;
	const int ret_err_unitid	=1;
	const int ret_err_server	=2;
	
	
	Simulator(std::string ip, int portNumber);
	
	/* Simulation Control */
	int connect();
	void disconnect();
	void pause();
	void resume();
	
	/* Interface */
	int getHandle(std::string name);
	int getPingTime(simInt*pingTime)const;
	int getLastCmdTime()const;
	
	/* Sensors */
	int readProximitySensor(simxInt sensorHandle, simxUChar *state, float *coord)const;
	int getVisionSensorImage(simxInt sensorHandle,simxUChar** image,simInt res[2])const;
	int getFloatSignal(std::string signalName,simxFloat signalValue[],int operationMode)const;
	
	/* Actuators */
	int setJointTargetVelocity(simxInt jointHandle, float velocity);
	
	/* Ground truth */
	int getObjectPosition   (simxInt sensorHandle, float *coord)const;
	int getObjectOrientation(simxInt sensorHandle, float *coord,simxInt relativeTo=-1)const;
	int getObjectVelocity   (simxInt sensorHandle,simxFloat*linVel,simxFloat*angVel)const;
	int getJointPosition    (simxInt jointHandle , float *coord)const;
	
};
#endif // SIMULATOR_HPP
