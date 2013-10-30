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



void LegStatusCallback(const std_msgs::Char::ConstPtr& msg);
void RobotTwistCallback(const geometry_msgs::Twist::ConstPtr& msg);


bool CurrentlyMoving = true;											// flag of whether serial controller returns that its currenly moving
int STATE = 0;
bool SHUTDOWN = false;													// flag to shutdown while loop
SpiderRobot::MyArray PosArray;											// ROS message to publish

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
	
	
}

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

void RobotTwistCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
	printf("Twist Status feedback:", twist->linear.x);
	
	
	
	
}// end RobotTwistCallback
