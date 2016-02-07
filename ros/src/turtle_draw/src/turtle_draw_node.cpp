#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// #include <signal.h>
// #include <termios.h>
#include <stdio.h>

#include "parson.h"

// ---------------------------------------------------------

class TurtleDraw {
public:
  TurtleDraw();
  void loop();

private:

  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher twist_pub_;
};

// ---------------------------------------------------------

TurtleDraw::TurtleDraw():
  linear_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
}

void TurtleDraw::loop()
{
  // char c;
  bool dirty=false;

  // get the console in raw mode
  // tcgetattr(kfd, &cooked);
  // memcpy(&raw, &cooked, sizeof(struct termios));
  // raw.c_lflag &=~ (ICANON | ECHO);
  // // Setting a new line, then end of file
  // raw.c_cc[VEOL] = 1;
  // raw.c_cc[VEOF] = 2;
  // tcsetattr(kfd, TCSANOW, &raw);
  //
  // puts("Reading from keyboard");
  // puts("---------------------------");
  // puts("Use arrow keys to move the turtle.");


  for(;;)
  {
    // get the next event from the keyboard
    // if(read(kfd, &c, 1) < 0) {
    //   perror("read():");
    //   exit(-1);
    // }

    linear_ = angular_ = 0;
    // ROS_DEBUG("value: 0x%02X\n", c);

    ROS_DEBUG("LEFT");
    angular_ = 1.0;
    dirty = true;
    // ROS_DEBUG("RIGHT");
    // angular_ = -1.0;
    // ROS_DEBUG("UP");
    // linear_ = 1.0;
    // ROS_DEBUG("DOWN");
    // linear_ = -1.0;


    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    if (dirty == true) {
      twist_pub_.publish(twist);
      dirty = false;
    }
  }


  return;
}

// ---------------------------------------------------------


// int kfd = 0;
// struct termios cooked, raw;

// void quit(int sig)
// {
//   // tcsetattr(kfd, TCSANOW, &cooked);
//   ros::shutdown();
//   exit(0);
// }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_draw");
  TurtleDraw turtleDraw;

  // signal(SIGINT,quit);

  turtleDraw.loop();

  return(0);
}
