#include <stdio.h>
#include <signal.h>
#include <fstream>
#include <unistd.h>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

#include "json/json.h"

#include "vector2d.h"

#define LOCATION_EPSILON 0.01
#define ANGLE_EPSILON 0.005
#define SPEED 3.0

void quit(int sig) {
  ROS_INFO("quit");
  ros::shutdown();
  exit(0);
}


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
  // void loop();

private:

  ros::NodeHandle nh_;
  ros::Publisher twist_pub_;
  ros::Subscriber pose_sub_;
  Json::Value instructions_;

  Vector2D currentLocation;
  double currentAngle;

  Vector2D nextGoal;
  int navIndex;

  void turtlePoseCB(const turtlesim::Pose::ConstPtr& msg);

  void getNextGoal() {
    nextGoal.x = instructions_[navIndex][0].asDouble();
    nextGoal.y = instructions_[navIndex][1].asDouble();
    ++navIndex;
  }

  bool done() {
    return (navIndex >= instructions_.size());
  }
};

// ---------------------------------------------------------

TurtleDraw::TurtleDraw(Json::Value instructions):
  instructions_(instructions),
  currentLocation(0,0),
  nextGoal(0,0),
  navIndex(0)
{
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
  pose_sub_ = nh_.subscribe("/turtle1/pose", 1, &TurtleDraw::turtlePoseCB, this);
  getNextGoal();
}



void TurtleDraw::turtlePoseCB(const turtlesim::Pose::ConstPtr& msg) {
  ROS_INFO_STREAM("new pose, theta:" << msg->theta);
  currentLocation.x = msg->x;
  currentLocation.y = msg->y;
  currentAngle = msg->theta;

  geometry_msgs::Twist twist;
  Vector2D delta = nextGoal - currentLocation;

  ROS_INFO_STREAM(
    "heading: " << currentAngle
    << ", direction to goal: " << delta.angle()
    << ", delta length: " << delta.length()
  );

  if (delta.length() < LOCATION_EPSILON) {
    // we've arrived at the next nav point

    if (done()) {
      // this was the last navigation point, we are done!
      quit(0);
    }

    getNextGoal();

  } else {

    twist.angular.z = (delta.angle() - currentAngle) * SPEED;

    // if deviated too much from direction, stop and turn
    if (std::abs(delta.angle() - currentAngle) > ANGLE_EPSILON) {
      twist.linear.x = 0;
    } else {
      // otherwise, move
      twist.linear.x = delta.length() * SPEED;
    }

    ROS_INFO_STREAM("x: " << twist.linear.x
      << ", z: " << twist.angular.z
      << ", heading: " << currentAngle
      << ", direction to goal: " << delta.angle()
      << ", delta length: " << delta.length()
    );
    twist_pub_.publish(twist);
  }
}

// // TODO: this needs to move in with turtlePoseCB
// void TurtleDraw::loop() {
//
//   // rate at which we'll send instructions
//   ros::Rate rate(10); // hz
//   rate.sleep();
//
//   geometry_msgs::Twist twist;
//
//   // Iterate over sequence of instructions
//   for ( int i = 0; i < instructions_.size(); ++i ) {
//     ROS_DEBUG_STREAM(i << ": " << instructions_[i]);
//     Vector2D nextGoal(instructions_[i][0].asDouble(),
//       instructions_[i][1].asDouble());
//     Vector2D delta = nextGoal - currentLocation;
//
//     while (delta.length() > LOCATION_EPSILON) {
//
//       // we also turn towards the goal
//       twist.angular.z = delta.angle() - currentAngle;
//
//       // if deviated too much from direction, stop and turn
//       if (std::abs(delta.angle() - currentAngle) > ANGLE_EPSILON) {
//         twist.linear.x = 0;
//       } else {
//         // otherwise, move
//         twist.linear.x = delta.length();
//       }
//
//       ROS_INFO_STREAM("x: " << twist.linear.x
//         << ", z: " << twist.angular.z
//         << ", heading: " << currentAngle
//         << ", direction to goal: " << delta.angle()
//         << ", delta length: " << delta.length()
//       );
//       twist_pub_.publish(twist);
//
//       rate.sleep();
//       delta = nextGoal - currentLocation;
//
//     }
//
//     // we've arrived at the next nav point
//   }
//
//   // we've arrived at the last nav point
//
//   return;
// }

// ---------------------------------------------------------

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
  ros::spin();

  return(0);
}
