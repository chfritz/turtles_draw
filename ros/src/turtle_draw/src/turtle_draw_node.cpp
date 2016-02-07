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
  TurtleDraw(Json::Value nav_points);
  void loop();

private:

  ros::NodeHandle nh_;
  ros::Publisher twist_pub_;
  Json::Value nav_points_;

};

// ---------------------------------------------------------

TurtleDraw::TurtleDraw(Json::Value nav_points):
  nav_points_(nav_points)
{
  // nh_.param("scale_angular", a_scale_, a_scale_);
  // nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
}

void TurtleDraw::loop() {

  // rate at which we'll send instructions
  ros::Rate rate(1); // hz
  rate.sleep();

  // Iterate over sequence of instructions (#TODO: rename nav_points if we keep
  // this semantics)
  for ( int i = 0; i < nav_points_.size(); ++i ) {
    ROS_DEBUG_STREAM(i << ": " << nav_points_[i]);

    geometry_msgs::Twist twist;
    if (nav_points_[i].isMember("linear")) {

      twist.linear.x = nav_points_[i]["linear"].asDouble();

    } else if (nav_points_[i].isMember("angular_deg")) {

      twist.angular.z =
      nav_points_[i]["angular_deg"].asDouble() * (M_PI / 180.0);

    } else {

      ROS_WARN_STREAM("unknown drawing instruction: " << nav_points_[i]);

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
  Json::Value nav_points = Json::Value();
  file_in >> nav_points;

  // let's go
  TurtleDraw turtleDraw(nav_points);
  turtleDraw.loop();

  return(0);
}
