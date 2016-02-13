#include <stdio.h>
#include <signal.h>
#include <fstream>
#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
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
  // TurtleDraw(Json::Value instructions);
  TurtleDraw();

private:

  ros::NodeHandle nh_;
  ros::Publisher twist_pub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber path_sub_;
  // Json::Value instructions_;

  /** the path to draw */
  nav_msgs::Path::ConstPtr path_;
  std::vector<geometry_msgs::PoseStamped>::const_iterator nextPoseStamped_;
  bool active;

  Vector2D currentLocation_;
  double currentAngle_;

  Vector2D nextGoal_;

  void turtlePoseCB(const turtlesim::Pose::ConstPtr& msg);

  void drawPathCB(const nav_msgs::Path::ConstPtr& msg);

  void getNext();

  bool done() {
    // return (navIndex >= instructions_.size());
    return (nextPoseStamped_ == path_->poses.end());
  }
};

// ---------------------------------------------------------

// TurtleDraw::TurtleDraw(Json::Value instructions):
// instructions_(instructions),
TurtleDraw::TurtleDraw():
  currentLocation_(0,0),
  nextGoal_(0,0),
  active(false)
  // navIndex(0)
{
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
  pose_sub_ = nh_.subscribe("/turtle1/pose", 1, &TurtleDraw::turtlePoseCB, this);
  path_sub_ = nh_.subscribe("/draw/path", 1, &TurtleDraw::drawPathCB, this);
}


/** Handle new pose message from turtle. This is, in a way, the inner loop of
our controller because new poses are what triggers behavior change. */
void TurtleDraw::turtlePoseCB(const turtlesim::Pose::ConstPtr& msg) {
  ROS_DEBUG_STREAM("new pose, theta:" << msg->theta);
  currentLocation_.x = msg->x;
  currentLocation_.y = msg->y;
  currentAngle_ = msg->theta;

  if (active) {

    geometry_msgs::Twist twist;

    Vector2D delta = nextGoal_ - currentLocation_;
    double angle_diff = delta.angle() - currentAngle_;

    ROS_INFO_STREAM(
      "rel. angle to goal: " << angle_diff
      << " , distance to goal: " << delta.length()
    );

    if (delta.length() < LOCATION_EPSILON) {
      // we've arrived at the next nav point

      if (done()) {
        // this was the last navigation point, we are done!
        // quit(0);
        active = false;
      }

      getNext();

    } else {

      // determine angular velocity to set
      if (angle_diff > M_PI) {
        angle_diff -= 2 * M_PI;
      } else if (angle_diff < -M_PI) {
        angle_diff += 2 * M_PI;
      }
      twist.angular.z = angle_diff * SPEED;

      // if deviated too much from direction, stop and turn
      if (std::abs(angle_diff) > ANGLE_EPSILON) {
        twist.linear.x = 0;
      } else {
        // otherwise, move
        twist.linear.x = delta.length() * SPEED;
      }

      ROS_DEBUG_STREAM("x: " << twist.linear.x
        << ", z: " << twist.angular.z
        << ", heading: " << currentAngle_
        << ", direction to goal: " << delta.angle()
        << ", delta length: " << delta.length()
      );
      twist_pub_.publish(twist);
    }
  }
}

/** handler for new /draw/path messages, coming from app. These contain new
shapes to draw. This is the outer loop of the controller. */
void TurtleDraw::drawPathCB(const nav_msgs::Path::ConstPtr& msg) {

  ROS_INFO_STREAM("new shape received");
  path_ = msg;
  nextPoseStamped_ = path_->poses.begin();
  active = true;
  getNext();
}

/** utility function to load the next way point into the inner loop controller */
void TurtleDraw::getNext() {
  // nextGoal_.x = instructions_[navIndex][0].asDouble();
  // nextGoal_.y = instructions_[navIndex][1].asDouble();
  // ++navIndex;
  if (active) {
    // TODO: create a setter in Vector2D for this (from Point)
    nextGoal_.x = nextPoseStamped_->pose.position.x;
    nextGoal_.y = nextPoseStamped_->pose.position.y;
    nextPoseStamped_++;
  }
  ROS_INFO_STREAM("next: " << nextGoal_.x << ", " << nextGoal_.y);

  // for (std::vector<geometry_msgs::PoseStamped>::const_iterator
  //   iterator = msg->poses.begin();
  //   iterator != msg->poses.end(); iterator++) {
  //
  //     geometry_msgs::PoseStamped pose = *iterator;
  //     ROS_INFO_STREAM(pose.pose.position.x << " " << pose.pose.position.y);
  // }
}



// ---------------------------------------------------------

int main(int argc, char** argv) {

  ros::init(argc, argv, "turtle_draw");
  signal(SIGINT,quit);
  ROS_INFO("START");

  // ROS_INFO("loading shape from file %s", argv[1]);
  // /* these files are just arrays of instruction with each instruction specifying
  //    a linear and/or angular_deg velocity, where the latter is just like angular
  //    but in degree rather than radian, which is easier for humans when
  //    specifying stars */
  // std::ifstream file_in(argv[1]);
  // Json::Value instructions = Json::Value();
  // file_in >> instructions;

  // let's go
  TurtleDraw turtleDraw;
  ros::spin();

  return(0);
}
