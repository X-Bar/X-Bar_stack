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

**********************************************************************************************************************/

#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "geometry_msgs/Twist.h"
#include <SpiderRobot/MyArray.h>										// to publish joint angles

#include <signal.h>

#include "SpiderRobot/InverseK.h"
#include "SpiderRobot/TransferFrame.h"
#include "SpiderRobot/SpiderConstants.h"
#include "SpiderRobot/AnglesToJoints.h"

void LegStatusCallback(const std_msgs::Char::ConstPtr& msg);
void RobotTwistCallback(const geometry_msgs::Twist::ConstPtr& msg);
void LegStatusCallback(const std_msgs::Char::ConstPtr& msg);
void shutdownHandler(int s);
short int WaitForDone(void);

using namespace SpiderRobotConstants;									// stores leg common leg positions

bool CurrentlyMoving = true;											// flag of whether serial controller returns that its currenly moving
int STATE = 0;															// for state machine
bool SHUTDOWN = false;													// flag to shutdown while loop
SpiderRobot::MyArray PosArray;											// ROS message to publish
short int LegGroupTurn = 0;											// leg group 0 or 1's turn to move

int main(int argc, char **argv)
{
	printf("Starting SpiderMovementController_pubsub \n");
	
	ros::init(argc, argv, "SpiderMovementController");					// start ROS connection
	ros::NodeHandle nh;													// make node handle
	// make publishing object and advertise
	ros::Publisher SpiderRobotMain_pub = nh.advertise<SpiderRobot::MyArray>("MyArray", 100);
	// make subscribing object for feedback
	ros::Subscriber LegStatus_sub = nh.subscribe("LegStatus", 1, LegStatusCallback);
	// make subscribing object for movement commands
	ros::Subscriber RobotTwist_sub = nh.subscribe("RobotTwist", 1, RobotTwistCallback);
	
	int LegAngs[3] = {0};												// temp for holding leg angles
	
	// Start up and stand
	while(ros::ok() && !SHUTDOWN)
	{
		switch(STATE)
		{
		  case 0: // start 
		  {
			//~ InverseKinematics(LegUpup_Car_Leg, LegAngs);			// find joint angles from general leg position
			PosArray = Angles2Joints(2, LegUpUp_Ang_Leg, PosArray);		// move all leg groups
			PosArray.speed = 100;										// do first move slowly
			//~ PosArray = Angles2Joints(2, LegAngs, PosArray);				// move all leg groups
			SpiderRobotMain_pub.publish(PosArray);						// publish command
			usleep(1*1000*1000);
			SpiderRobotMain_pub.publish(PosArray);						// publish first command twice
			CurrentlyMoving = true;										// change status to moving as command was given
			STATE = 1;
			PosArray.speed = 300;
			break;
	  	  }
		  case 1:
		  {
			InverseKinematics(LegUpOut_Car_Leg, LegAngs);				// find joint angles from general leg position
			PosArray = Angles2Joints(2, LegAngs, PosArray);				// move all leg groups
			SpiderRobotMain_pub.publish(PosArray);						// publish command
			CurrentlyMoving = true;
			STATE = 2;
			break;
		  }
		  case 2:
		  {
			InverseKinematics(LegDownOut_Car_Leg, LegAngs);				// find joint angles from general leg position
			PosArray = Angles2Joints(2, LegAngs, PosArray);				// move all leg groups
			SpiderRobotMain_pub.publish(PosArray);						// publish command
			CurrentlyMoving = true;
			STATE = 3;
			break;
		  }
		  case 3:
		  {
			InverseKinematics(LegUpIn_Car_Leg, LegAngs);				// find joint angles from general leg position
			PosArray = Angles2Joints(1, LegAngs, PosArray);				//  move leg group 1
			SpiderRobotMain_pub.publish(PosArray);						// publish command
			CurrentlyMoving = true;
			STATE = 4;
			break;
		  }
		  case 4:
		  {
			InverseKinematics(LegDownIn_Car_Leg, LegAngs);				// find joint angles from general leg position
			PosArray = Angles2Joints(1, LegAngs, PosArray);				//  move leg group 1
			SpiderRobotMain_pub.publish(PosArray);						// publish command
			CurrentlyMoving = true;
			STATE = 5;
			break;
		  }
		  case 5:
		  {
			InverseKinematics(LegUpIn_Car_Leg, LegAngs);				// find joint angles from general leg position
			PosArray = Angles2Joints(0, LegAngs, PosArray);				//  move leg group 0
			SpiderRobotMain_pub.publish(PosArray);						// publish command
			CurrentlyMoving = true;
			STATE = 6;
			break;
		  }
		  case 6:
		  {
			InverseKinematics(LegDownIn_Car_Leg, LegAngs);				// find joint angles from general leg position
			PosArray = Angles2Joints(0, LegAngs, PosArray);				//  move leg group 0
			SpiderRobotMain_pub.publish(PosArray);						// publish command
			CurrentlyMoving = true;
			SHUTDOWN = true;											// stop while loop, startup is done, go to spin() now
			break;
		  }
		  default:
		  {
			PosArray.command = 1;										// command 1 is exit for serial controller
			SpiderRobotMain_pub.publish(PosArray);						// publish command
			usleep(1000*1000);
			SHUTDOWN = true;											// stop while loop
			break; break;												// exit
		  }
		}// end switch(STATE)
		usleep(1000*1000);
		ros::spinOnce();
		ros::spinOnce();
		WaitForDone();
	}// while(ros::ok() && !SHUTDOWN)
	
	usleep(1000*1000);
	ros::spin();														// wait for callbacks, they do all the work
	
	return 0;
}

void RobotTwistCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
	//~ printf("Twist Status feedback: %f", twist->linear.x);
	ROS_INFO("Twist Status feedback: %f", twist->linear.x);
	usleep(1000*1000);
	// calculate movement
	
	
	
}// end RobotTwistCallback

/***********************************************************************************************************************
void LegStatusCallback(const std_msgs::Char::ConstPtr& msg)
* callback that recieves leg status and set globle bool variable 
* CurrentlyMoving so that other functions know
***********************************************************************************************************************/
void LegStatusCallback(const std_msgs::Char::ConstPtr& msg)
{
	//~ printf("Leg Status feedback: %c\n", msg->data);
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
short int WaitForDone(void)
* forces code wait till function LegStatusCallback() declares robot
* is ready to move
***********************************************************************************************************************/
short int WaitForDone(void)
{
	//for(int i = 0; i < 1000; i++)
	while(CurrentlyMoving && ros::ok() && !SHUTDOWN)
	{
		ros::spinOnce();												// Check for leg status msg
		usleep(100*1000);												// wait 10th of a second
	}
	return 0;
}// end WaitForDone()

void shutdownHandler(int s)
{
	ROS_INFO("SHUTDOWN COMMAND DETECTED...\n");
	printf("SHUTDOWN COMMAND DETECTED...\n");
	SHUTDOWN = true;
}
