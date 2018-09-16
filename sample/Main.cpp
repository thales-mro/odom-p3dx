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

using namespace std;
Gnuplot gp;
std::vector<std::pair<double, double> > robot_pose_over_time;
std::vector<std::pair<double, double>> obstacle_points;

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

	robot_pose_over_time.push_back(std::make_pair(double(robot_pos[0]), double(robot_pos[1])));
	gp << "set xrange [-7:7]\nset yrange [-7:7]\n";
	gp << "plot '-' with lines title 'gtTrajectory', '-' with points title 'Obstacles'\n";
	//gp.send1d(xy_pts_A);
	gp.send1d(robot_pose_over_time);
	gp.send1d(obstacle_points);
}


int main(int argc, char *argv[]){

	cout << "ou" << endl;
	//std::cin.sync_with_stdio(false);
	//std::ios::sync_with_stdio(false);
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

	static bool ad_infinitum=false;
	static unsigned int i_max=3000*56;
	static unsigned int sleep_us=2000;
	
	for (int i=0;(gSignalStatus!=SIGINT) && (ad_infinitum||i<i_max);++i){
		robot->updateSensors();
		std::vector<float> sonarReadings = robot->getSonarReadings();
		//cout << "--------------------------------------------" << endl;
		//for(float sonarReading : sonarReadings) {
		//	std::cout << sonarReading << std::endl;
		//}
		std::vector<simxFloat> robot_pos = robot->getRobotPos();
		robot->updatePose();
		//std::cout << "Robot gt pos: " << robot_pos[0] << " " << robot_pos[1] << " " << robot_pos[2] << std::endl;
		simxUChar state;
		simxFloat coord[3];

		//std::cout << "Sensor 1 reading" << coord[0] << " " << coord[1] << " " << coord[2] << std::endl;

		//robot->move(4, 0);
		robot->writeGT();
		//robot->writeSonars();
		//extApi_sleepMs(50);
		usleep(sleep_us);

		printGnuplot(robot_pos, robot->getRobotOrn()[2], sonarReadings);
	}
	std::cout<<std::endl<<"Disconnecting..."<<std::endl;
	vrep->disconnect();
	gp << std::endl;
	exit(0);
}
