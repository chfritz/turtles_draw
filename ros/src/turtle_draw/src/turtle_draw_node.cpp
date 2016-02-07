#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// #include <signal.h>
// #include <termios.h>
#include <stdio.h>
#include <fstream>
#include <unistd.h>

#include "json/json.h"

// ---------------------------------------------------------

class TurtleDraw {
public:
  TurtleDraw(Json::Value instructions);
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
  // nh_.param("scale_angular", a_scale_, a_scale_);
  // nh_.param("scale_linear", l_scale_, l_scale_);

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

    } else if (instructions_[i].isMember("angular_deg")) {

      twist.angular.z =
      instructions_[i]["angular_deg"].asDouble() * (M_PI / 180.0);

    } else {

      ROS_WARN_STREAM("unknown drawing instruction: " << instructions_[i]);

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
  std::ifstream file_in(argv[1]);
  Json::Value instructions = Json::Value();
  file_in >> instructions;

  // let's go
  TurtleDraw turtleDraw(instructions);
  turtleDraw.loop();

  return(0);
}
