#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
// #include <termios.h>
#include <stdio.h>
#include <fstream>
#include <unistd.h>

#include "json/json.h"

// ---------------------------------------------------------

/** simple class that simply translates instructions from json into message for
    the turtle1/cmd_vel topic */
class TurtleDraw {
public:

  /** @param instructions is a JSON array of objects, each of which
      can contain linear and/or angular_deg fields, specifying linear and
      angular velocity
  */
  TurtleDraw(Json::Value instructions);

  /** loop just loops through the instructions and send them to the turtle (via
      the cmd_vel topic) */
  void loop();

private:

  ros::NodeHandle nh_;
  ros::Publisher twist_pub_;
  Json::Value instructions_;

};

// ---------------------------------------------------------

TurtleDraw::TurtleDraw(Json::Value instructions):
  instructions_(instructions)
{
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
}

void TurtleDraw::loop() {

  // rate at which we'll send instructions
  ros::Rate rate(1); // hz
  rate.sleep();

  // Iterate over sequence of instructions
  for ( int i = 0; i < instructions_.size(); ++i ) {
    ROS_DEBUG_STREAM(i << ": " << instructions_[i]);

    geometry_msgs::Twist twist;
    if (instructions_[i].isMember("linear")) {

      twist.linear.x = instructions_[i]["linear"].asDouble();

    }
    if (instructions_[i].isMember("angular_deg")) {

      twist.angular.z =
      instructions_[i]["angular_deg"].asDouble() * (M_PI / 180.0);

    }

    twist_pub_.publish(twist);

    rate.sleep();
  }

  return;
}

// ---------------------------------------------------------

void quit(int sig) {

  ROS_INFO("quit");
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "turtle_draw");
  signal(SIGINT,quit);
  ROS_INFO("START");

  ROS_INFO("loading shape from file %s", argv[1]);
  /* these files are just arrays of instruction with each instruction specifying
     a linear and/or angular_deg velocity, where the latter is just like angular
     but in degree rather than radian, which is easier for humans when
     specifying stars */
  std::ifstream file_in(argv[1]);
  Json::Value instructions = Json::Value();
  file_in >> instructions;

  // let's go
  TurtleDraw turtleDraw(instructions);
  turtleDraw.loop();

  return(0);
}
