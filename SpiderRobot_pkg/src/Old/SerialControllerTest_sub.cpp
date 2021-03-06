/***********************************************************************************************************************
FILENAME:   SerialController.cpp
AUTHORS:    Cody L. Lundberg based on code from Christopher D. McMurrough (base_controller.cpp for IGVC)

DESCRIPTION:
ROS interface node serial commands. Recieves topic, converts to serial commands and sends them. Currently in use with SSC-32 servo controller

PUBLISHES:  NA
SUBSCRIBES: "cmd_vel" geometry_msgs/TwistStamped, "autonomous_enabled" std_msgs/Bool
SERVICES:   NA

REVISION HISTORY:
05.06.2013   CDM     Christopher D. McMurrough original file creation
05.17.2013           Cody L. Lundberg reuse. Focus on making code more general purpose.
***********************************************************************************************************************/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "SpiderRobot/My2Num.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/TwistStamped.h>
#include <iostream>

#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <math.h>

#define SERIAL_PACKET_START_CHAR 0xAA
#define SERIAL_PACKET_LENGTH 0x08
#define PI 3.14159265

bool SHUTDOWN = false;

unsigned char MOTOR_1A_COMMAND = 0;
unsigned char MOTOR_1B_COMMAND = 0;
unsigned char MOTOR_2A_COMMAND = 0;
unsigned char MOTOR_2B_COMMAND = 0;

ros::Time LAST_COMMAND_TS;

unsigned char SAFETY_LIGHT_STATE = 0;
const int SAFETY_LIGHT_CYCLE = 20;
int AUTONOMY_STATE = 0;

// define the timeout duration
double TIMEOUT_SECONDS = 0.5;

// serial port handler
int serialPort;

void motionCommandCallback(const SpiderRobot::My2Num::ConstPtr& msg);
int CheckPos(int i, int Pos);
int openSerialPort(char* portName);
void closeSerialPort(int serialPort);
void writeSerialBytes(int serialPort, unsigned char* data, int numBytes);
void makeTeensyPacket(unsigned char* buffer, unsigned char m1a_val, unsigned char m1b_val, unsigned char m2a_val, unsigned char m2b_val, unsigned char light); // change to make type of packet thing
void sendBaseCommand(int serialPort, unsigned char m1a_val, unsigned char m1b_val, unsigned char m2a_val, unsigned char m2b_val, unsigned char light); // sends commands via serial
void pollSerialPort(int serialPort); // read port
void shutdownHandler(int s);

/***********************************************************************************************************************
int main(int argc, char **argv)
program entry point, expects the serial port name and baud rate as command line arguments
***********************************************************************************************************************/
int main(int argc, char **argv)
{
    printf("Starting\n");
    // store the serial port parameters
    char* portName;

    // validate and parse the command line arguments
    /*
    if(argc != 2)
    {
        ROS_WARN("USAGE: %s <port_name> \n", argv[0]);
        return 0;
    }
    */
    portName = "/dev/ttyUSB0";

    // attempt to open the serial port
    serialPort = openSerialPort(portName);

    // check to see if we connected successfully
    if(serialPort == -1)
    {
        printf("unable to open serial port %s \n", portName);
        return(0);
    }
    else
    {
	printf("serial port opened: %s \n", portName);
    }


    // set up the shutdown handler
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = shutdownHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // initialize the ROS node
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle nh;

    // shutdown the device until a command is received
    LAST_COMMAND_TS = ros::Time::now() - ros::Duration(TIMEOUT_SECONDS);
    //stopMotors(serialPort);

    // set the loop rate to 20 Hz
    ros::Rate loop_rate(50);

    // store the light blinking counter
    int blinkCounter = 0;

    // start the subscriber
    ros::Subscriber motionCommandSubscriber = nh.subscribe("My2Num", 10, motionCommandCallback);
    ros::spin();

    // listen for message until program termination
    while(ros::ok() && !SHUTDOWN)
    {
        // issue motor commands if the status is OK
	/*
        if(ros::Time::now().toSec() - LAST_COMMAND_TS.toSec() < ros::Duration(TIMEOUT_SECONDS).toSec())
        {
            sendBaseCommand(serialPort, MOTOR_1A_COMMAND, MOTOR_1B_COMMAND, MOTOR_2A_COMMAND, MOTOR_2B_COMMAND, SAFETY_LIGHT_STATE);
        }
        else
        {
            //stopMotors(serialPort);
            ROS_WARN("COMMAND TIMEOUT: motors stopped!");
        }
	*/

        // perform one iteration of message checking
        ros::spinOnce();

        // check for received serial data
        pollSerialPort(serialPort);

        // sleep to maintain the loop rate
        loop_rate.sleep();
    }


    // stop the motors
    //sendBaseCommand(serialPort, 90, 90, 90, 90, 0);

    // close the serial port
    closeSerialPort(serialPort);
}

//void motionCommandCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
void motionCommandCallback(const SpiderRobot::My2Num::ConstPtr& msg)
{
    printf("Recieving data...\n");
    char buffer[20];
    int Pos = msg->pos;
    int Cha = msg->cha;
    int i = 0;

    Pos = CheckPos(Cha, Pos);

    int uSecPos = 11.11111*((float)Pos) + 1500.0;			// convert to useconds of duty cycle
    ROS_INFO("uSecPos: [%d]", uSecPos);				// print
    int n = sprintf(buffer, "#%dP%d", Cha, uSecPos);		// convert to string
    buffer[n] = 13;
    printf("bits %d\n", n);
    for(i = 0 ; i < n ; i++)
	printf("%c", buffer[i]);
    printf("\n");

    //int result = write(serialPort, buffer, n+1);			// send to ssc-32
    //printf("result: %d\n", result);




    LAST_COMMAND_TS = ros::Time::now();
}

/***********************************************************************************************************************
void CheckPos(int *Pos)
Takes array of desired angle positions, checks them against known bad angles for safety.
***********************************************************************************************************************/
int CheckPos(int i, int Pos)
{
	if( i == 0 || i == 3 || i == 6 || i == 9 || i == 12 || i == 15 )	// first joint
	{
		if(Pos < 1050)
		{
			Pos = 1050;
			printf("angle %d changed\n", i);
		}
		else if(Pos > 1950)
		{
			Pos = 1950;
			printf("angle %d changed\n", i);
		}
	}
	if( i == 1 || i == 4 || i == 7 || i == 10 || i == 13 || i == 16 )	// second joint
	{
		if(Pos < 1250)
		{
			Pos = 1250;
			printf("angle %d changed\n", i);
		}
		else if(Pos > 2250)
		{
			Pos = 2250;
			printf("angle %d changed\n", i);
		}
	}
	if( i == 2 || i == 5 || i == 8 || i == 11 || i == 14 || i == 17 )	// third joint
	{
		if(Pos < 600)
		{
			Pos = 600;
			printf("angle %d changed\n", i);
		}
		else if(Pos > 1750)
		{
			Pos = 1750;
			printf("angle %d changed\n", i);
		}
	}
	return Pos;
}// end CheckPos

int openSerialPort(char* portName)
{
    // store the file descriptor for the serial port
    int fd;

    // attempt to open the port
    fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

    // return -1 if we are unable to open the port
    if(fd == -1)
    {
        return -1;
    }
    else
    {
        // clear any existing file descriptor flags
        fcntl(fd, F_SETFL, 0);

        // create a structure to store the port settings
        struct termios port_settings;

        // set the baud rates
        cfsetispeed(&port_settings, B115200);
        cfsetospeed(&port_settings, B115200);

	/*
        // set no parity, stop bits, data bits
        port_settings.c_cflag &= ~PARENB;
        port_settings.c_cflag &= ~CSTOPB;
        port_settings.c_cflag &= ~CSIZE;
        port_settings.c_cflag |= CS8;
	*/
	port_settings.c_iflag = IGNBRK | IGNPAR;
        port_settings.c_oflag = 0;
        port_settings.c_cflag = B115200 | CREAD | CS8 | CLOCAL;
        port_settings.c_lflag = 0;

        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, &port_settings);

        memset(&port_settings, 0, sizeof(port_settings));
        tcgetattr(fd, &port_settings);

        // set the port to use all 8 bits
        port_settings.c_iflag &= ~ISTRIP;

        // apply the settings to the port
        tcsetattr(fd, TCSANOW, &port_settings);

        // set the non blocking functionality
        fcntl(fd, F_SETFL, O_NONBLOCK);

        // return the file descriptor
        return(fd);
    }
}

/***********************************************************************************************************************
void closeSerialPort(int serialPort)
close the given serial port
***********************************************************************************************************************/
void closeSerialPort(int serialPort)
{
    tcflush(serialPort, TCIOFLUSH);
    close(serialPort);
}

/***********************************************************************************************************************
void writeSerialBytes(int serialPort, unsigned char* data, int numBytes)
write the byte data to the serial port
***********************************************************************************************************************/
void writeSerialBytes(int serialPort, unsigned char* data, int numBytes)
{
    int result = write(serialPort, data, numBytes);
}

/***********************************************************************************************************************
void makeTeensyPacket(unsigned char* buffer, unsigned char m1a_val, unsigned char m1b_val, unsigned char m2a_val, unsigned char m2b_val, unsigned char light)
create a base command packet from the given values, returning the result in buffer
***********************************************************************************************************************/
void makeTeensyPacket(unsigned char* buffer, unsigned char m1a_val, unsigned char m1b_val, unsigned char m2a_val, unsigned char m2b_val, unsigned char light)
{
    // initialize the packet
    buffer[0] = SERIAL_PACKET_START_CHAR;
    buffer[1] = SERIAL_PACKET_LENGTH;
    buffer[2] = m1a_val;
    buffer[3] = m1b_val;
    buffer[4] = m2a_val;
    buffer[5] = m2b_val;
    buffer[6] = light;
    buffer[7] = 0x00;

    // compute the checksum
    for(int i = 0; i < SERIAL_PACKET_LENGTH - 1; i++)
    {
        buffer[SERIAL_PACKET_LENGTH - 1] = buffer[SERIAL_PACKET_LENGTH - 1] ^ buffer[i];
    }
}

/***********************************************************************************************************************
void sendBaseCommand(int serialPort, unsigned char m1a_val, unsigned char m1b_val, unsigned char m2a_val, unsigned char m2b_val, unsigned char light)
create a packet from the given parameters and send to the serial port
***********************************************************************************************************************/
void sendBaseCommand(int serialPort, unsigned char m1a_val, unsigned char m1b_val, unsigned char m2a_val, unsigned char m2b_val, unsigned char light)
{
    // create the outgoing packet buffer
    unsigned char packet[SERIAL_PACKET_LENGTH];

    // create the packet
    makeTeensyPacket(packet, m1a_val, m1b_val, m2a_val, m2b_val, light);

    // write the packet
    writeSerialBytes(serialPort, packet, SERIAL_PACKET_LENGTH);
}

/***********************************************************************************************************************
void pollSerialPort(int serialPort)
processed received data from the serial port
***********************************************************************************************************************/
void pollSerialPort(int serialPort)
{
    const int bufferSize = 100;
    static unsigned char buff[bufferSize];
    int n;

    // attempt to read bytes from the port
    n = read(serialPort, buff, bufferSize);

    // print any received bytes to terminal
    if(n > 0)
    {
        ROS_INFO("RECEIVED TTY RESPONSE...");
    }
}

/***********************************************************************************************************************
void shutdownHandler(int s)
send a stop motor command to the controller
***********************************************************************************************************************/
void shutdownHandler(int s)
{
    ROS_INFO("SHUTDOWN COMMAND DETECTED...");
    SHUTDOWN = true;
}





