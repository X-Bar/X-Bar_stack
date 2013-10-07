#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <signal.h>
#include <termios.h>
#include <stdio.h>

//#define KEYCODE_R 0x43 
//#define KEYCODE_L 0x44
//#define KEYCODE_U 0x41
//#define KEYCODE_D 0x42
//#define KEYCODE_Q 0x71

struct termios cooked, raw;
int kfd = 0;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "tester");
  ros::NodeHandle n;
  ros::Publisher commandsend_pub = n.advertise<std_msgs::String>("commandsend", 1000);
  //ros::Rate loop_rate(10);
  std_msgs::String cds_pub;

  signal(SIGINT,quit);

  char c;
  
  
  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  bool inputyes = false;
  
  puts("Enter Keys");

  int count = 0;
  while (ros::ok())
  {    
    cds_pub.data = {'\0', '\0'};

    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    cds_pub.data = c;
    char command[11] ={'\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'};
    
    switch(c)
    {
      case 'w': 
        strcpy(command, "Forward");
        inputyes = true;
        break;
      case 'a': 
        strcpy(command, "Left");
        inputyes = true;
        break;
      case 's': 
        strcpy(command, "Backward");
        inputyes = true;
        break;
      case 'd': 
        strcpy(command, "Right");
        inputyes = true;
        break;
    }

    fprintf(stdout, "%c %s\n", c, command);

    if(inputyes == true)
    {
      commandsend_pub.publish(cds_pub);    
      inputyes = false;
    }
    
    




    //ros::spinOnce();

    //loop_rate.sleep();
    ++count;
  }
}
