/*My includes*/

/* Debugging */
//#include "prettyprint.hpp"
#ifdef DEBUG_BUILD
#define DBG(x)		do{if(1){std::cerr<<#x<<":\t["<<x<<"]"<<std::endl;}}while(0);
#else
#define DBG(x)		;
#endif

//#include <QCoreApplication>
#include "Robot.hpp"
#include "Simulator.hpp"
#include <iostream>
#include <csignal>
#include <unistd.h>
#include <vector>
#include "/home/thales/gnuplot-iostream/gnuplot-iostream.h"

/* Signal Handler */
namespace{
	volatile std::sig_atomic_t gSignalStatus;
}
void signal_handler(int signal){
	gSignalStatus=signal;
}

extern "C" {
	#include "extApi.h"
	#include "v_repLib.h"
}


using namespace std;

double calculateAngleDiff(double posAngle, double negAngle) {
	double diff;

	diff = (M_PI - posAngle) + (M_PI + negAngle);

	return diff;
}


double convertSensorPosToAngle(int pos) {
	int angle;
	switch(pos) {
		case 0 :
			angle = 90;
		break;
		case 1:
			angle = 50;
		break;
		case 2:
			angle = 30;
		break;
		case 3:
			angle = 10;
		break;
		case 4:
			angle = -10;
		break;
		case 5:
			angle = -30;
		break;
		case 6:
			angle = -50;
		break;
		case 7:
			angle = -90;
		break;
	}
	std::cout << "angle pos: " << angle << std::endl;
	double convAngle = double(angle*M_PI)/180.0;
	std::cout << convAngle << std::endl;
	return convAngle;
}

std::pair<double, double> convertSensorDistToGlobalFrame(simxFloat x_robot, simxFloat y_robot, simxFloat robotOrn, double sensorAngle, simxFloat distance, simxFloat robotRadius) {
	double x_coord = x_robot + (robotRadius + distance)*cos(robotOrn + sensorAngle);
	double y_coord = y_robot + (robotRadius + distance)*sin(robotOrn + sensorAngle);
	std::pair<double, double> position;
	position = std::make_pair(x_coord, y_coord);
	return position;
	//return make_pair<double, double>(x_coord, y_coord);
}

Gnuplot gp;
std::vector<std::pair<double, double> > robot_pos_gt;
std::vector<std::pair<double, double>> obstacle_points;
std::vector<std::pair<double, double>> robot_odometry;

void printGnuplot(vector<simxFloat> robot_pos, float robot_orn, vector<simxFloat> sensorReadings) {
	
	if(robot_pos[2] == 0.0f)
		return;

	for(int i = 0; i < 8; i++) {
		if(sensorReadings[i] != -1) {
			cout << i << endl;
			obstacle_points.push_back(convertSensorDistToGlobalFrame(robot_pos[0], robot_pos[1], robot_orn, convertSensorPosToAngle(i), sensorReadings[i], 0.0975));
		}
	}
	//for(double alpha=0; alpha<1; alpha+=1.0/24.0) {
	//	double theta = alpha*2.0*3.14159;
	//	xy_pts_B.push_back(std::make_pair(cos(theta), sin(theta)));
	//}

	robot_pos_gt.push_back(std::make_pair(double(robot_pos[0]), double(robot_pos[1])));
	robot_odometry.push_back(std::make_pair(double(robot_pos[0]), double(robot_pos[1]) + 1));
	gp << "set xrange [-7:7]\nset yrange [-7:7]\n";
	gp << "plot '-' with lines title 'gtTrajectory', '-' with points title 'Obstacles', '-' with lines title 'odometry trajectory'\n";
	//gp.send1d(xy_pts_A);
	gp.send1d(robot_pos_gt);
	gp.send1d(obstacle_points);
	gp.send1d(robot_odometry);
}


int main(int argc, char *argv[]){
	DBG("Debugging is ON");
	
	std::signal(SIGINT, signal_handler);
	
	Robot *robot;
	Simulator *vrep = new Simulator("127.0.0.1", 25000);
	cout << "tryna connect" << endl;
	for(;;){
		try{
			if (vrep->connect() ==-1){
				std::cout << "Failed to Connect" << std::endl;
				return 0;
			}
			break;
		}catch(...){
			std::cout<<"Could not connect. Retrying..."<<std::endl;
		}
	}
	robot = new Robot(vrep, "Pioneer_p3dx");
	std::cout << "------------------------------LOAD FINISHED------------------------------" << std::endl;
	
	/*Main Loop*/
	int lMotorHandle = vrep->getHandle("Pioneer_p3dx_leftMotor");
	std::cout << lMotorHandle << std::endl;
	int rMotorHandle = vrep->getHandle("Pioneer_p3dx_rightMotor");
	vrep->setJointTargetVelocity(lMotorHandle, 3);
	vrep->setJointTargetVelocity(rMotorHandle, 3);
	int sonarHandling = vrep->getHandle("Pioneer_p3dx_ultrasonicSensor1");
	simxFloat encoderRight, encoderLeft, auxRight, auxLeft;

	static bool ad_infinitum=false;
	static unsigned int i_max=3000*56;
	static unsigned int sleep_us=20000;
	
	robot->updatePose();
	std::vector<simxFloat> starting_pos = robot->getRobotPos();
	simxFloat starting_orn = robot->getRobotOrn()[2];
	vrep->getJointPosition(lMotorHandle, &encoderLeft);
	vrep->getJointPosition(rMotorHandle, &encoderRight);

	for (int i=0;(gSignalStatus!=SIGINT) && (ad_infinitum||i<i_max);++i){
		robot->updateSensors();
		std::vector<float> sonarReadings = robot->getSonarReadings();
		robot->updatePose();
		std::vector<simxFloat> robot_pos = robot->getRobotPos();

		vrep->getJointPosition(lMotorHandle, &auxLeft);
		vrep->getJointPosition(rMotorHandle, &auxRight);

		double deltaThetaLeft = auxLeft - encoderLeft;
		double deltaThetaRight = auxRight - encoderRight;

		if(auxLeft > 3.0 && encoderLeft < 0) {
			deltaThetaLeft = calculateAngleDiff(auxLeft, encoderLeft);
		}
		if(encoderLeft > 3.0 && auxLeft < 0) {
			deltaThetaLeft = calculateAngleDiff(encoderLeft, auxLeft);
		}
		

		if(auxRight > 3.0 && encoderRight < 0) {
			deltaThetaRight = calculateAngleDiff(auxRight, encoderRight);
		}
		if(encoderRight > 3.0 && auxRight < 0) {
			deltaThetaRight = calculateAngleDiff(encoderRight, auxRight);
		}

		if(deltaThetaLeft != 0)
			cout << "Diff Value: " << deltaThetaLeft << endl; 
		
		encoderLeft = auxLeft;
		encoderRight = auxRight;

		simxUChar state;
		simxFloat coord[3];

		//std::cout << "Sensor 1 reading" << coord[0] << " " << coord[1] << " " << coord[2] << std::endl;
		robot->writeGT();
		usleep(sleep_us);

		//printGnuplot(robot_pos, robot->getRobotOrn()[2], sonarReadings);
	}
	std::cout<<std::endl<<"Disconnecting..."<<std::endl;
	vrep->disconnect();
	gp << std::endl;
	exit(0);
}
