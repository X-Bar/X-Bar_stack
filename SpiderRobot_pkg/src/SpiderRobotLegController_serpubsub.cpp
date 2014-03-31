#include <stdio.h>
#include "ros/ros.h"

#include <SpiderRobot_pkg/MyArray.h>									// to publish joint angles
#include "SpiderRobot_pkg/SpiderConstants.h"
#include "SpiderRobot_pkg/SpiderFunctions.h"
#include "SpiderRobot_pkg/AddTwoInts.h"

SpiderRobot_pkg::MyArray Angles2Joints(short int group, int Joints[3], SpiderRobot_pkg::MyArray PosArray);
float* TransferFrame(short int Mode,short int Leg, float BasePoints[]);
void LegStatusCallback(const std_msgs::Char::ConstPtr& msg);
void shutdownHandler(int s);
short int WaitForDone(void);


int main(int argc, char **argv)
{
	ROS_INFO("Starting SpiderLegController_serpubsub");
	ros::init(argc, argv, "SpiderLegController_serpubsub");
	ros::NodeHandle nh;
	
	ros::ServiceServer service = nh.advertiseService("LegController", MoveLeg);
	ros::spin();

}

bool MoveLeg(SpiderRobot_pkg::LegController::Request  &req, SpiderRobot_pkg::LegController::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

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