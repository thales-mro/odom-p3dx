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

Gnuplot gp;
//vector that stores robot`s x & y ground truth coordinates over time
std::vector<std::pair<double, double> > robot_pos_gt;
//vector that stores points identified by ultrasonic sensots
std::vector<std::pair<double, double>> obstacle_points;
//vector that stores robot`s x & y coordinates calculated through odometry
std::vector<std::pair<double, double>> robot_odometry;
double robot_odom_orn;
//cumulative sum on x axis coordinates(for odometry)
double sumX = 0;
//cumulative sum on y axis coordinates(for odometry)
double sumY = 0;
//cumulative sum on z axis rotation (for odometry)
double sumOrn = 0;

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
	double convAngle = double(angle*M_PI)/180.0;
	return convAngle;
}

std::pair<double, double> convertSensorDistToGlobalFrame(simxFloat x_robot, simxFloat y_robot, simxFloat robotOrn, double sensorAngle, simxFloat distance, simxFloat robotRadius) {
	double x_coord = x_robot + (robotRadius + distance)*cos(robotOrn + sensorAngle);
	double y_coord = y_robot + (robotRadius + distance)*sin(robotOrn + sensorAngle);
	std::pair<double, double> position;
	position = std::make_pair(x_coord, y_coord);
	return position;
}

void calculatesOdomAndPlot(vector<simxFloat> robot_pos, float robot_orn, vector<simxFloat> sensorReadings, double deltaThetaLeft, double deltaThetaRight) {
	
	//adds ground truth info from this iteration	
	robot_pos_gt.push_back(std::make_pair(double(robot_pos[0]), double(robot_pos[1])));

	//calculates point cloud according to ultrasonic sensor readings
	for(int i = 0; i < 8; i++) {
		if(sensorReadings[i] != -1) {
			obstacle_points.push_back(convertSensorDistToGlobalFrame(robot_pos[0], robot_pos[1], robot_orn, convertSensorPosToAngle(i), sensorReadings[i], 0.0975));
		}
	}

	//calculates Odometry from kinematic model
	double R = 0.0975;
	double L = 0.36205;
	double deltaS = R*(deltaThetaRight + deltaThetaLeft)/2;
	double deltaTheta = R*(deltaThetaRight - deltaThetaLeft)/(2*L);
	pair<double, double> pose = robot_odometry[0];

	sumX += deltaS*cos(sumOrn + (deltaTheta/2));
	sumY += deltaS*sin(sumOrn + (deltaTheta/2));
	sumOrn += deltaTheta;
	if(sumOrn < -(M_PI*2)) {
		sumOrn += (M_PI*2);
	}
	if(sumOrn > (M_PI*2)) {
		sumOrn -= (M_PI*2);
	}
	robot_odometry.push_back(std::make_pair(sumX + pose.first, sumY + pose.second));
	
	//plots data
	gp << "set xrange [-10:10]\nset yrange [-10:10]\n";
	gp << "plot '-' with lines title 'gtTrajectory', '-' with points title 'Obstacles', '-' with lines title 'odometry trajectory'\n";
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
	static unsigned int sleep_us=200000;
	
	robot->updatePose();
	std::vector<simxFloat> starting_pos = robot->getRobotPos();
	simxFloat starting_orn = robot->getRobotOrn()[2];
	robot_odometry.push_back(make_pair<double, double>(starting_pos[0], starting_pos[1]));
	robot_odom_orn = starting_orn;
	sumOrn = robot_odom_orn;
	vrep->getJointPosition(lMotorHandle, &encoderLeft);
	vrep->getJointPosition(rMotorHandle, &encoderRight);

	double vRapido = 3.0;
	double vMedio = 2.0;
	double vLento = 1.0;

	vrep->setJointTargetVelocity(lMotorHandle, 0);
	vrep->setJointTargetVelocity(rMotorHandle, 0);

	for (int i=0;(gSignalStatus!=SIGINT) && (ad_infinitum||i<i_max);++i){
		robot->updateSensors();
		std::vector<float> sonarReadings = robot->getSonarReadings();
		robot->updatePose();
		//it gets robot`s ground truth
		std::vector<simxFloat> robot_pos = robot->getRobotPos();

		//it gets encoder`s positions and calculates angle variation from one timestamp to another
		vrep->getJointPosition(lMotorHandle, &auxLeft);
		vrep->getJointPosition(rMotorHandle, &auxRight);

		double deltaThetaLeft = auxLeft - encoderLeft;
		double deltaThetaRight = auxRight - encoderRight;

		//it corrects angle`s discontinuity
		if(auxLeft > M_PI/2 && encoderLeft < 0) {
			deltaThetaLeft = -1*calculateAngleDiff(auxLeft, encoderLeft);
		}
		if(encoderLeft > M_PI/2 && auxLeft < 0) {
			deltaThetaLeft = calculateAngleDiff(encoderLeft, auxLeft);
		}
		if(auxRight > M_PI/2 && encoderRight < 0) {
			deltaThetaRight = -1*calculateAngleDiff(auxRight, encoderRight);
		}
		if(encoderRight > M_PI/2 && auxRight < 0) {
			deltaThetaRight = calculateAngleDiff(encoderRight, auxRight);
		}

		encoderLeft = auxLeft;
		encoderRight = auxRight;

		robot->writeGT();
		robot->writeSonars();

		//"dumb" controller
		/*if((sonarReadings[3] != -1 && sonarReadings[3] < 0.4) || (sonarReadings[4] != -1 && sonarReadings[4] < 0.4)) {
			if(sonarReadings[7] > 0.3 || sonarReadings[7] == -1) {
				vrep->setJointTargetVelocity(lMotorHandle, vRapido);
				vrep->setJointTargetVelocity(rMotorHandle, 0);
			}
			else if(sonarReadings[0] > 0.3 || sonarReadings[0] == -1) {
				vrep->setJointTargetVelocity(lMotorHandle, 0);
				vrep->setJointTargetVelocity(rMotorHandle, vRapido);
			}
		}
		else {
			vrep->setJointTargetVelocity(lMotorHandle, vRapido);
			vrep->setJointTargetVelocity(rMotorHandle, vRapido);
		}*/
		
		calculatesOdomAndPlot(robot_pos, robot->getRobotOrn()[2], sonarReadings, deltaThetaLeft, deltaThetaRight);

		if(abs(deltaThetaLeft) < 0.001 && abs(deltaThetaRight) < 0.001) {
			vrep->setJointTargetVelocity(lMotorHandle, -1*vRapido);
			vrep->setJointTargetVelocity(rMotorHandle, -1*vRapido);
			usleep(100);
		}
		else {
			if((sonarReadings[3] > 0.0 && sonarReadings[3] < 0.15) || (sonarReadings[4] > 0.0 && sonarReadings[4] < 0.15)) {
				vrep->setJointTargetVelocity(lMotorHandle, 0);
				vrep->setJointTargetVelocity(rMotorHandle, 0);
				if(sonarReadings[7] < 0.0 || sonarReadings[7] > 1) {
					vrep->setJointTargetVelocity(lMotorHandle, vLento);
					vrep->setJointTargetVelocity(rMotorHandle, 0);
				}
				else if(sonarReadings[0] < 0.0 || sonarReadings[0] > 1) {
					vrep->setJointTargetVelocity(lMotorHandle, 0);
					vrep->setJointTargetVelocity(rMotorHandle, vLento);
				}
				else 
					cout << "Parouuuuuuuuuuuuuuuuuuuuu" << endl;
			}
			else {
				if(sonarReadings[5] > 0 && sonarReadings[5] < 0.35){
					vrep->setJointTargetVelocity(lMotorHandle, 0);
					vrep->setJointTargetVelocity(rMotorHandle, vLento);
				}
				else {
					if(sonarReadings[7] > 0.0 && sonarReadings[7] < 0.20) {
						vrep->setJointTargetVelocity(lMotorHandle, vRapido);
						vrep->setJointTargetVelocity(rMotorHandle, vRapido);
					}
					else if(sonarReadings[7] > 0.20) {
						vrep->setJointTargetVelocity(lMotorHandle, vMedio);
						vrep->setJointTargetVelocity(rMotorHandle, vLento);
					}
					else if(sonarReadings[7] < 0.0){
						vrep->setJointTargetVelocity(lMotorHandle, vRapido);
						vrep->setJointTargetVelocity(rMotorHandle, vLento);
					}
				}
			}
		}
		cout << "-----------------------------------------" << endl;
		for(int i = 0; i < 8; i++) {
			cout << i << ": " << sonarReadings[i] << endl;
		}
		usleep(sleep_us);
	}
	std::cout<<std::endl<<"Disconnecting..."<<std::endl;
	vrep->disconnect();
	gp << std::endl;
	exit(0);
}
