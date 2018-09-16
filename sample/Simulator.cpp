#include "Simulator.hpp"

Simulator::Simulator(std::string ip, int portNumber){
	this->id = -1;
	this->ip = ip;
	this->portNumber = portNumber;
}

/* Simulation Control */
int Simulator::connect(){
	id = 1;
	// Connection to server
	id = simxStart(ip.c_str(), portNumber, true, true, 2000, 5);
	if(id==-1) throw std::string("Unable to connect to V-REP Server");
	return id;
}
void Simulator::disconnect(){
	if(id!=-1)simxFinish(id);
}
void Simulator::pause(){
	if(id!=-1)simxPauseCommunication(id,0);
}
void Simulator::resume(){
	if(id!=-1)simxPauseCommunication(id,1);
}

/* Interface */
int Simulator::getHandle(std::string name){
	if(id==-1){return ret_err_unitid;}
	int handle=-1;
	if(simxGetObjectHandle(id, name.c_str(), &handle, simx_opmode_oneshot_wait) != simx_return_ok)
		throw std::string("Unable to receive handle");
	return handle;
}

int Simulator::getPingTime(simInt*pingTime)const{
	if(id==-1){return ret_err_unitid;}
	if(simxGetPingTime(id,pingTime) != simx_return_ok){
		return ret_err_server;
	}
	return ret_ok;
}

int Simulator::getLastCmdTime()const{
	if(id==-1){return ret_err_unitid;}
	return simxGetLastCmdTime(id);
}

/* Sensors */
int Simulator::readProximitySensor(simxInt sensorHandle, simxUChar *state, float *coord)const{
	if(id==-1){return ret_err_unitid;}
	simxReadProximitySensor(id,sensorHandle,state,coord,NULL,NULL,simx_opmode_streaming);
	return ret_ok;
}
int Simulator::getVisionSensorImage(simxInt sensorHandle,simxUChar** img,simInt res[2])const{
	if(id==-1){return ret_err_unitid;}
	simxUChar options=0;
	if(simxGetVisionSensorImage(id,sensorHandle,res,img,options,simx_opmode_streaming) == simx_error_noerror)
		return ret_ok;
	return ret_err_server;
}
int Simulator::getFloatSignal(std::string signalName, simxFloat signalValue[], simxInt operationMode)const{
	if(id==-1){return ret_err_unitid;}
	if(simxGetFloatSignal(id,signalName.c_str(),signalValue,operationMode) != simx_error_noerror){
		return ret_err_server;
	}
	return ret_ok;
}

/* Actuators */
int Simulator::setJointTargetVelocity(simxInt jointHandle, float velocity) {
	if(id==-1){return ret_err_unitid;}
	simxSetJointTargetVelocity(id, jointHandle, velocity, simx_opmode_streaming);
	return ret_ok;
}

/* Ground truth */
int Simulator::getObjectPosition(simxInt sensorHandle, simxFloat *coord)const{
	if(id==-1){return ret_err_unitid;}
	if(simxGetObjectPosition(id,sensorHandle,-1,coord,simx_opmode_streaming)==simx_return_ok)
		return ret_ok;
	return ret_err_server;
}
int Simulator::getObjectOrientation(simxInt sensorHandle, simxFloat *coord,simxInt relativeTo)const{
	if(id==-1){return ret_err_unitid;}
	if(simxGetObjectOrientation(id,sensorHandle,relativeTo,coord,simx_opmode_streaming)==simx_return_ok)
		return ret_ok;
	return ret_err_server;
}
int Simulator::getObjectVelocity(simxInt sensorHandle,simxFloat*linVel,simxFloat*angVel)const{
	if(id==-1){return ret_err_unitid;}
	if(simxGetObjectVelocity(id,sensorHandle,linVel,angVel,simx_opmode_streaming)==simx_return_ok)
		return ret_ok;
	return ret_err_server;
}
int Simulator::getJointPosition(simxInt jointHandle, float *coord)const{
	if(id==-1){return ret_err_unitid;}
	simxGetJointPosition(id,jointHandle,coord,simx_opmode_streaming);
	return ret_ok;
}

