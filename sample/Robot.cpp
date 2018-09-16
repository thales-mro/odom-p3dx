#include "Robot.hpp"
#include <iostream>

/* class Robot{ */
Robot::Robot(Simulator *sim, std::string name){
	this->sim = sim;
	this->name = name;
	this->handle = sim->getHandle(name);
	for(int i = 0; i < NUM_SONARS; i++) {
		sonarReadings.push_back(0.0f);
	}
	//this->collection_handle = sim->getCollectionHandle(name+"_collection");
	
	if(LOG){
		FILE *data;
		data=fopen("gt.txt","wt");		if(data!=NULL){fclose(data);}
		data=fopen("sonar.txt","wt");	if(data!=NULL){fclose(data);}
	}
	
	/* Get handles */{
		/* Sensors */{
			/* Sonar sensors: 1 handle/sensor; Sensor name: Pioneer_p3dx_ultrasonicSensor[1-16] */
				simxChar sensorName[31];
				for(int i = 0; i < NUM_SONARS; i++){
					sprintf(sensorName,"%s%d","Pioneer_p3dx_ultrasonicSensor",i+1);
					try {
						sonarHandle[i] = sim->getHandle(sensorName);
						if(sonarHandle[i] == sim->ret_err_unitid ){std::cout<< "Error on connecting to sensor " << i <<std::endl;}
						else{std::cout<< "Connected to sensor\n";}
					}
					catch(...) {
						std::cout << "Failed to connect to ultrasonic sensor " << i << std::endl;
					}
				}
			/* Vision sensor */
				try {
					vsensHandle[0] = sim->getHandle("Pioneer_p3dx_visionSensor");
					if(vsensHandle[0] == sim->ret_err_unitid){std::cout<<"Error on connecting to vision sensor "<<std::endl;}
					else{std::cout<<"Connected to vision sensor\n";}
				}
				catch(...) {
					std::cout << "Failed to connect to vision sensor" << std::endl;
				}
		}
		/* Actuators */{
			/* Left/Right motor */
				motorHandle[0] = sim->getHandle("Pioneer_p3dx_leftMotor");
				motorHandle[1] = sim->getHandle("Pioneer_p3dx_rightMotor");
				std::cout << "Left motor: "<<  motorHandle[0] << std::endl;
				std::cout << "Right motor: "<<  motorHandle[1] << std::endl;
		}
		/* Model */{
			/* Left/Right wheel */
				wheelHandle[0] = sim->getHandle("Pioneer_p3dx_leftWheel");
				wheelHandle[1] = sim->getHandle("Pioneer_p3dx_rightWheel");
				std::cout << "Left wheel: "<< wheelHandle[0] << std::endl;
				std::cout << "Right wheel: "<< wheelHandle[1] << std::endl;
		}
	}
	
	/* Get positions */{
		/* Robot */{
			sim->getObjectPosition(handle,robot_pos);
			sim->getObjectOrientation(handle,robot_orn);
			robot_t0_xyt[0]=robot_pos[0];
			robot_t0_xyt[1]=robot_pos[1];
			robot_t0_xyt[2]=robot_orn[2];
			for(int i=0;i<3;i++){robot_pos_lst[i]=robot_pos[i];}
		}
		/* Motor */{
			sim->getJointPosition(motorHandle[0],&motor_pos[0]);
			sim->getJointPosition(motorHandle[1],&motor_pos[1]);
			//std::cout << motor_pos[0] << std::endl;
			//std::cout << motor_pos[1] << std::endl;
		}
		/* GyroSensor */{
			sim->getFloatSignal("gyroZ",&gyros_reading,simx_opmode_streaming);
		}
	}
}


/* Update */
void Robot::updateSensors(){
	/* Sensors */
	{
		/* Sonar sensors */
		for(int i=0; i<NUM_SONARS; ++i){
			simxUChar state;		// sensor state
			simxFloat coord[3];		// sonar_reading_normed point coordinates [only z matters]
			/* simx_opmode_streaming -> Non-blocking mode */
			/* read the proximity sensor
				* sonar_reading_normionState: pointer to the state of sonar_reading_normion (0=nothing sonar_reading_normed)
				* sonar_reading_normedPoint: pointer to the coordinates of the sonar_reading_normed point (relative to the frame of reference of the sensor) */
			if (sim->readProximitySensor(sonarHandle[i],&state,coord) == sim->ret_ok){
				if(state>0)	{sonarReadings[i] = coord[2];}
				else		{sonarReadings[i] = -1;}
			}
		}
		/* Normalized Sonar sensors */
		for(int i=0;i<NUM_SONARS;++i){
			float dist=sonarReadings[i];
			if(dist>0 && dist<sonar_dtct_max_dist){
				if(dist<sonar_dtct_min_dist){dist=sonar_dtct_min_dist;}
				sonar_reading_norm[i]=1-((dist-sonar_dtct_min_dist)/(sonar_dtct_max_dist-sonar_dtct_min_dist));
			}
			else{sonar_reading_norm[i]=0;}
		}
	/* GyroSensor */
		sim->getFloatSignal("gyroZ",&gyros_reading,simx_opmode_buffer);
	}
	/* Encoders */
	{
		/* Left/Right motor */
		motor_pos_lst[0] = motor_pos[0];
		motor_pos_lst[1] = motor_pos[1];
		/* Get the motor_pos data */
		if(sim->getJointPosition(motorHandle[0], &motor_pos[0]) == sim->ret_ok){
			//std::cout<<"ok left enconder"<<motor_pos[0]<<std::endl;	//left
		}
		if(sim->getJointPosition(motorHandle[1], &motor_pos[1]) == sim->ret_ok){
			//std::cout<<"ok right enconder"<<motor_pos[1]<<std::endl;	//right
		}
	}
}

void Robot::updatePose(){
	for(int i=0; i<3; ++i){
		robot_pos_lst[i]= robot_pos[i];
		robot_orn_lst[i]= robot_orn[i];
	}
	/* Get the robot current position and orientation */
	sim->getObjectPosition   (handle,robot_pos);
	sim->getObjectOrientation(handle,robot_orn);

	for(int i = 0; i < 3; i++) {
		ret_robot_orn[i] = robot_orn[i];
		ret_robot_pos[i] = robot_pos[i];
	}
}


/* Actuate */
void Robot::move(float vLeft, float vRight)const{
	sim->setJointTargetVelocity(motorHandle[0], vLeft);
	sim->setJointTargetVelocity(motorHandle[1], vRight);
}
void Robot::stop()const{
	move(0,0);
}
void Robot::drive(double vLinear, double vAngular)const{
	move(vLToDrive(vLinear,vAngular),vRToDrive(vLinear,vAngular));
}
double Robot::vRToDrive(double vLinear, double vAngular)const{
	return (((2*vLinear)+(L*vAngular))/2*R);
}
double Robot::vLToDrive(double vLinear, double vAngular)const{
	return (((2*vLinear)-(L*vAngular))/2*R);
}

/* Output */
void Robot::printPose()const{
	std::cout<<"[" <<robot_pos[0] <<", " <<robot_pos[1] <<", " <<robot_orn[2] <<"]" <<std::endl;
}
void Robot::writeGT()const{
	/* file format: robot_pos[3] robot_pos_lst[3] robot_orn[3] motor_pos[2] motor_pos_lst[2] */
	if(LOG){
		FILE *data=fopen("gt.txt", "at");
		if(data!=NULL){
			for(int i=0; i<3; ++i){fprintf(data, "%.2f\t",ret_robot_pos[i]);}
			//for(int i=0; i<3; ++i){fprintf(data, "%.2f\t",robot_pos_lst[i]);}
			for(int i=0; i<3; ++i){fprintf(data, "%.2f\t",ret_robot_orn[i]);}
			//for(int i=0; i<2; ++i){fprintf(data, "%.2f\t",motor_pos[i]);}
			//for(int i=0; i<2; ++i){fprintf(data, "%.2f\t",motor_pos_lst[i]);}
			fprintf(data, "\n");
			fflush(data);
			fclose(data);
		}
		else{std::cout << "Unable to open file";}
    }
}
void Robot::writeSonars()const{
	if(LOG){
		FILE *data=fopen("sonar.txt", "at");
		if(data!=NULL){
			for(int i=0; i<NUM_SONARS; ++i){fprintf(data, "%.2f\t",sonarReadings[i]);}
			fprintf(data,"\n");
			fflush(data);
			fclose(data);
		}
	}
}



/* }; */



