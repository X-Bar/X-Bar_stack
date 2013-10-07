/***********************************************************************************************************************
FILENAME:   SpiderRobotMain_pub.cpp
AUTHORS:    Cody L. Lundberg

DESCRIPTION:
Planning node for SpiderRobot package

PUBLISHES:  "MyArray" SpiderRobot/MyArray
SUBSCRIBES: NA
SERVICES:   NA

REVISION HISTORY:
05.20.2013   CDM     Cody L. Lundberg original file creation

***********************************************************************************************************************
SpiderRobot Legs
6 legs, 3 DoF each. 18 servos, 18 channels
MyArray PosArray.data[18] makes angle positions for all 18 servos

Left side
Leg one: channels 0, 1, 2
Leg Two: channels 3, 4, 5
Leg Three: channels 6, 7, 8

Right side
Leg one: channels 9, 10, 11
Leg two: channels 12, 13, 14
Leg three: channels 15, 16, 17

Leg group 0
Leg one: channels 0, 1, 2
Leg two: channels 12, 13, 14
Leg Three: channels 6, 7, 8

Leg group 1
Leg one: channels 9, 10, 11
Leg Two: channels 3, 4, 5
Leg Three: channels 6, 7, 8

***********************************************************************************************************************/

#include "ros/ros.h"
#include "std_msgs/Char.h"
//#include <geometry_msgs/TwistStamped.h>
#include <iostream>

#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>

#include "SpiderRobot/TransferFrame.h"
#include <SpiderRobot/MyArray.h>

void Angles2Joints(short int group, int Joints[3]);
void LegStatusCallback(const std_msgs::Char::ConstPtr& msg);
void shutdownHandler(int s);
short int WaitForDone(void);
float* TransferFrame(short int Mode,short int Leg, float BasePoints[]);
void MultiplyMat(float A[4][4], float B[4][1], float C[4][1]);//,int N, int L, int M);
short int InverseKinematics(float BasePoints[3], int LegAng[3]);


bool SHUTDOWN = false;									// flag to shutdown while loop
SpiderRobot::MyArray PosArray;							// ROS message to publish
bool CurrentlyMoving = true;							// flag of whether serial controller returns that its currenly moving
int STATE = 0;


int main(int argc, char **argv)
{
	printf("Starting SpiderRobotMain_pub \n");
	
	ros::init(argc, argv, "SpiderRobotMain");			// start ROS connection
	ros::NodeHandle nh;									// make node handle
	// make publishing object and advertise
	ros::Publisher SpiderRobotMain_pub = nh.advertise<SpiderRobot::MyArray>("MyArray", 100);
	// make subscribing object for feedback
	ros::Subscriber LegStatus_sub = nh.subscribe("LegStatus", 1, LegStatusCallback);

	int i = 0;											// counter
	short int MySpeed = 300;							// speed for joints to move
	
	int* LegArray;
	PosArray.command = 0;								// command number. 0 is move all legs
	PosArray.speed = 300;								// speed for joints to move
	PosArray.size = 18;									// size of data array
	ros::Rate loop_rate(.2);							// while loop rate
	
	// number of general positions, old version
	//  side/leg/joint  CurrentlyMoving   L/1/1      L/1/3   L/2/2      L/3/1     L/3/3     R/1/2     R/2/1     R/2/3     R/3/2                      
	//  side/leg/joint          L/1/2     L/2/1     L/2/3     L/3/2     R/1/1     R/1/3     R/2/2     R/3/1    R/3/3                     
	//  channel      Lside	 0    1    2    3    4    5    6    7     8  RS9  10   11   12    13  14    15  16    17
	int AllLegsUpUp[] = 	{0,  63,  -80,  0,  63,  -80,  0,  63,  -80,  0,  63,  -80,  0,  63,  -80,  0,  63,  -80};
	int AllLegsHome[] = 	{0,   0,    0,  0,   0,    0,  0,   0,    0,  0,   0,    0,  0,   0,    0,  0,   0,    0};
	int LegG0Down[] = 	{0, -22,  -15,  0,   0,    0,  0, -22,  -15,  0,   0,    0,  0, -22,  -15,  0,   0,    0};
	int LegG0Up[] = 	{0,  27,   25,  0,   0,    0,  0,  27,   25,  0,   0,    0,  0,  27,   25,  0,   0,    0};
	int LegG1Down[] = 	{0,   0,    0,  0, -22,  -15,  0,   0,    0,  0, -22,  -15,  0,   0,    0,  0, -22,  -15};
	int LegG1Up[] = 	{0,   0,    0,  0,  27,   27,  0,   0,    0,  0,  27,   25,  0,   0,    0,  0,  27,   25};

	// number of general positions, new version. Use with function Angles to Joints
	//			J0  J1  J2
	int LegsUpUp[] =	{0, 63, -80};					// position for leg group all straight up
	int LegsHome[] =	{0, 0, 0};						// leg all at zero
	int LegsDown[] =	{0, -30, -35};					// legs in a standing down position
	int LegsUp[] =		{0, 27, 25};					// legs in a lefted position

	printf("Starting system loop..\n");
	STATE = 0; 											// set state to start
	
	/////////////
	
	float pos1[] = {0.2032, 0.2032, -.11}; 
	float* pos2;
	pos2 = TransferFrame(0,0, pos1);
	
	printf("\n\n %f %f %f \n\n", pos2[0], pos2[1], pos2[2] );
	
	int LegAngs[3] = {0};
	InverseKinematics(pos2, LegAngs);
	
	
	return 0;
	/////////////
	
	while(ros::ok() && !SHUTDOWN)
	{
		switch(STATE)
		{
		  case 0: // start 
		  {
			Angles2Joints(2, LegsUpUp);					// move all leg groups upup
			SpiderRobotMain_pub.publish(PosArray);		// publish command
			usleep(1*1000*1000);
			SpiderRobotMain_pub.publish(PosArray);		// publish first command twice
			usleep(1*1000*1000);
			STATE = 1;
			break;
	  	  }
		  case 1:
		  {
			Angles2Joints(2, LegsUp);					// move all leg groups upup
			SpiderRobotMain_pub.publish(PosArray);		// publish command
			STATE = 2;
			break;
		  }
		  case 2:
		  {
			Angles2Joints(2, LegsDown);					// move all leg groups upup
			SpiderRobotMain_pub.publish(PosArray);		// publish command
			STATE = 3;
			break;
		  }
		  case 3:
		  {
			Angles2Joints(1, LegsUp);					// move all leg groups upup
			SpiderRobotMain_pub.publish(PosArray);		// publish command
			STATE = 4;
			break;
		  }
		  case 4:
		  {
			Angles2Joints(2, LegsDown);					// move all leg groups upup
			SpiderRobotMain_pub.publish(PosArray);		// publish command
			STATE = 5;
			break;
		  }
		  case 5:
		  {
			Angles2Joints(0, LegsUp);					// move all leg groups upup
			SpiderRobotMain_pub.publish(PosArray);		// publish command
			STATE = 2;
			break;
		  }


		  default:
		  {
			PosArray.command = 1;						// command 1 is exit for serial controller
			SpiderRobotMain_pub.publish(PosArray);		// publish command
			SHUTDOWN = true;							// stop while loop
			break; break;								// exit
		  }
		}// end switch(STATE)
		usleep(1000*1000);
		ros::spinOnce();
		ros::spinOnce();
		WaitForDone();									// wait for completetion
	}// end while loop





	printf("SHUTDOWN COMMAND DETECTED...\n\n");
}// end main

void shutdownHandler(int s)
{
	ROS_INFO("SHUTDOWN COMMAND DETECTED...\n");
	printf("SHUTDOWN COMMAND DET	ECTED...\n");
	SHUTDOWN = true;
}

/***********************************************************************************************************************
void Angles2Joints(short int group, int Joints[3])
Takes array of desired leg joints and places them in PosArray.data for publishing
* int group will place the joints in the correct positions of PosArray.data for the desired leg group
* * group 0: places angles for leg group 0
* * group 1: places angles for leg group 1
* * group 2: places angles for leg group 0 and 1
***********************************************************************************************************************/
void Angles2Joints(short int group, int Joints[3])
{
	switch(group)
	{
	 case 0: // leg group 0
	 {
		PosArray.data[0] = Joints[0];
		PosArray.data[1] = Joints[1];
		PosArray.data[2] = Joints[2];
		PosArray.data[6] = Joints[0];
		PosArray.data[7] = Joints[1];
		PosArray.data[8] = Joints[2];
		PosArray.data[12] = Joints[0];
		PosArray.data[13] = Joints[1];
		PosArray.data[14] = Joints[2];
		break;
	 }
	 case 1: // leg group 1
	 {
		PosArray.data[3] = Joints[0];
		PosArray.data[4] = Joints[1];
		PosArray.data[5] = Joints[2];
		PosArray.data[9] = Joints[0];
		PosArray.data[10] = Joints[1];
		PosArray.data[11] = Joints[2];
		PosArray.data[15] = Joints[0];
		PosArray.data[16] = Joints[1];
		PosArray.data[17] = Joints[2];
		break;
	 } 
	 case 2: // all legs
	 {
		PosArray.data[0] = Joints[0];
		PosArray.data[1] = Joints[1];
		PosArray.data[2] = Joints[2];
		PosArray.data[6] = Joints[0];
		PosArray.data[7] = Joints[1];
		PosArray.data[8] = Joints[2];
		PosArray.data[12] = Joints[0];
		PosArray.data[13] = Joints[1];
		PosArray.data[14] = Joints[2];

		PosArray.data[3] = Joints[0];
		PosArray.data[4] = Joints[1];
		PosArray.data[5] = Joints[2];
		PosArray.data[9] = Joints[0];
		PosArray.data[10] = Joints[1];
		PosArray.data[11] = Joints[2];
		PosArray.data[15] = Joints[0];
		PosArray.data[16] = Joints[1];
		PosArray.data[17] = Joints[2];
		break;
	 }// end case 2
	}// end switch
	
}// end Angles2Joints()

void LegStatusCallback(const std_msgs::Char::ConstPtr& msg)
{
	printf("Leg Status feedback: %c\n", msg->data);
	switch(msg->data)
	{
	  case '.': // Not moving
	  {
		CurrentlyMoving = false;
		break;
	  }
	  case '+': // Moving
	  {
		CurrentlyMoving = true;
		break;
	  }
	}
}// end LegStatusCallback()

/***********************************************************************************************************************
int WaitForDone(void)

***********************************************************************************************************************/
short int WaitForDone(void)
{
	//for(int i = 0; i < 1000; i++)
	while(CurrentlyMoving && ros::ok() && !SHUTDOWN)
	{
		ros::spinOnce();								// Check for leg status msg
		usleep(100*1000);								// wait 10th of a second
	}
	return 0;
}// end WaitForDone()

/***********************************************************************************************************************
float* InverseKinematics(float BasePoints[3])
Takes array of leg end effector position in leg frame, transfers it into jointspace and returns
* output float LegAng[3]
* returns int = 0 if successful
***********************************************************************************************************************/
short int InverseKinematics(float BasePoints[3], int LegAng[3])
{
	// inverse k should be leg independent and not need modes
	// based on law of cosines
	// need to find theta0, theta1, and theta2
	float theta0, theta1, theta2;
	float D1;										// for inverse k
	float Ax;											// x and z points in xz plane
	float temp;											// holder
	float PI = 3.1416;
	// stage 1, find theta 0
	theta0 = atan2(BasePoints[1], BasePoints[0]);
	
	// stage 2, find Ax in xz plane
	Ax = sqrt( pow(BasePoints[0], 2) + pow(BasePoints[1], 2) );

	Ax = Ax - .03378;	// add offset for J1 to J2 distance
	
	// stage 3 find theta1 and theta2 from Az and BasePoints[2] or "Az"
	// D1 = ( Goal.x(2)^2 + Goal.y(2)^2 - L(1).leng^2 - L(2).leng^2 ) / (2*L(1).leng*L(2).leng); // from matlab
	D1 = pow(Ax, 2) + pow(BasePoints[2], 2) - pow(0.05715, 2) - pow(0.127, 2);
	D1 = D1 / (2.0*0.05715*0.127);
	
	// L(2).angdes =  atan2( real(-sqrt(1-D1^2)),D1 );
	temp = 1 - pow(D1,2);
	if(temp > 0) 
	{
		theta2 = atan2( -sqrt(temp), D1);
	}
	else
	{
		ROS_ERROR("Bad value (1 - pow(D1,2) > 0) = false:: %f\n", temp);
		ROS_INFO("Theta0: %f, Ax: %f, D1: %f, temp: %f\n", theta0, Ax, D1, temp);
		ROS_INFO("BasePoints: %f, %f, %f\n", BasePoints[0], BasePoints[1], BasePoints[2]);
		return 1;
	}
	
	// L(1).angdes =  atan2(Goal.y(2),Goal.x(2)) - atan2(L(2).leng*sin(L(2).angdes), L(1).leng + L(2).leng*cos(L(2).angdes))
	theta1 = atan2(BasePoints[2], Ax) - atan2(0.127*sin(theta2), 0.05715 + 0.127*cos(theta2));
	printf("Leg angles: %f, %f, %f\n", theta0*180.0/PI, theta1*180.0/PI, theta2*180.0/PI);
	
	LegAng[0] = (int)(theta0*180.0/PI);
	LegAng[1] = (int)(theta1*180.0/PI);
	LegAng[2] = (int)(theta2*180.0/PI);
	
	return 0;
}


