#pragma once
#ifndef ROBOT_HPP
#define ROBOT_HPP

/* Debugging */
//#include "prettyprint.hpp"
#ifdef DEBUG_BUILD
#define DBG(x)		do{if(1){std::cerr<<#x<<":\t["<<x<<"]"<<std::endl;}}while(0);
#else
#define DBG(x)		;
#endif

#include <fstream>
#include <iostream>
#include "Simulator.hpp"
#include <cmath>
#include <vector>

extern "C"{
	#include "extApi.h"
	#include "v_repLib.h"
}

/* Constants */
const bool LOG=true;
const bool vsens_enabled=1;

class Robot{
private:
	/* Model Data */
		const float L = 0.36205;						// distance between wheels
			// L = 0.381;	// given
		const float R = 0.0975;							// wheel radius
		const float motor_velocity_max=8;
		/* Sonars */							   	/*0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17*/
		const static int NUM_SONARS=16;
		const int sonar_ang_deg_can[NUM_SONARS  ]={ 90, 50, 30, 10,350,330,310,290,250,250,210,190,170,150,130,110};
		const float sonar_dtct_min_dist=0.2;
		const float sonar_dtct_max_dist=0.5;			// farther obstacles are not detected
		/* Vision Sensor */
		const int vsens_res_x=64;						// vision sensor X resolution (fixed in scene)
		const int vsens_res_y=64;						// vision sensor Y resolution (fixed in scene)
		
	/* Simulation Control */
		unsigned int ctrl_start=0;
		simxInt pingTime[1];
		simxInt sim_lct    ;
		simxInt sim_lct_lst;
	
	/* Simulation Handle */
		std::string name;
		Simulator *sim;
		simxInt handle;									// robot handle
		//simxInt collection_handle;					// robot collection handle
		/* Sensor Handle */
		simxInt sonarHandle[16];						// handle for sonars
		simxInt vsensHandle[1];							// handle for vision sensor
		/* Actuator Handle */
		simxInt motorHandle[2] = {0,0};					// [0]-> leftMotor [1]->rightMotor
	
	/* Sensor Readings */
		/* Sonars */
		std::vector<float> sonarReadings;
		float sonar_reading_norm[NUM_SONARS];			// readings in [(far)0 , 1(close)] interval
		/* Vision Sensor */
		//bool vsens_enabled=false;
		simxUChar* vsens_img[1];
		/* Joints / Encoder */
		simxFloat motor_pos[2]		= {0,0};			// motor encoder for left+right
		simxFloat motor_pos_lst[2]	= {0,0};			// motor last encoder for left+right
		/* GyroSensor */
		simxFloat gyros_reading;
	
		/* Ground truth */
		/* GT Handle */
		simxInt wheelHandle[2] = {0,0};
		/* GT Robot Position  */
		simxFloat robot_pos[3]		= {0,0,0};			// current robot position
		simxFloat robot_pos_lst[3]	= {0,0,0};			// last robot position
		simxFloat robot_orn[3]		= {0,0,0};			// current robot orientation
		simxFloat robot_orn_lst[3]	= {0,0,0};			// current robot orientation
		simxFloat robot_t0_xyt[3]	= {0,0,0};
		std::vector<simxFloat> ret_robot_pos{0,0,0};
		std::vector<simxFloat> ret_robot_orn{0,0,0};

		//simxFloat velocity[2] = {1,1};				// wheels' speed
	
	/* Odometry */
		simxFloat odm_raw_pos[2]= {0,0};				// vanilla odometry
		simxFloat odm_raw_orn[1]= {0};
		simxFloat odm_cps_pos[2]= {0,0};				// compass odometry
		simxFloat odm_cps_orn[1]= {0};
	
public:
		Robot(Simulator *sim, std::string name);
	/* Update */
		void updateSensors();
		void updatePose();
	
	/* Actuate */
		void move(float vLeft, float vRight)const;
		double vRToDrive(double vLinear, double vAngular)const;
		double vLToDrive(double vLinear, double vAngular)const;
		void drive(double vLinear, double vAngular)const;
		void stop()const;
	
	/* Output */
		void printPose()const;
		void writeGT()const;
		void writeSonars()const;

		std::vector<float> getSonarReadings() {return sonarReadings;}
		std::vector<float> getRobotPos() {return ret_robot_pos;}
		std::vector<float> getRobotOrn() {return ret_robot_orn;}

};

#endif // ROBOT_HPP
