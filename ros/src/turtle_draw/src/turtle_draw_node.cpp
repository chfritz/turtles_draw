
// system includes
#include <stdio.h>
#include <signal.h>
#include <fstream>
#include <unistd.h>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <turtlesim/Pose.h>

// other libraries
#include "json/json.h"

// local includes
#include "vector2d.h"

// euclidean distance tolerance for navigation points
#define LOCATION_EPSILON 0.01
// angular tolerance: above this angle we will stop moving to turn
#define ANGLE_EPSILON 0.005
// speed of the turtle (multiplier of propotional error in controller)
#define SPEED 3.0


// ---------------------------------------------------------

/** simple class that simply translates instructions from json into message for
    the turtle1/cmd_vel topic */
class TurtleDraw {
public:

  TurtleDraw();

private:

  ros::NodeHandle nh_;
  ros::Publisher twist_pub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber path_sub_;

  /** the path to draw */
  nav_msgs::Path::ConstPtr path_;
  std::vector<geometry_msgs::PoseStamped>::const_iterator nextPoseStamped_;
  bool active;
  Vector2D nextGoal_;

  Vector2D currentLocation_;
  double currentAngle_;

  // ---- methods

  void turtlePoseCB(const turtlesim::Pose::ConstPtr& msg);
  void drawPathCB(const nav_msgs::Path::ConstPtr& msg);
  void getNext();
  bool done() {
    return (nextPoseStamped_ == path_->poses.end());
  }
};

// ---------------------------------------------------------

TurtleDraw::TurtleDraw():
  currentLocation_(0,0),
  nextGoal_(0,0),
  active(false)
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

    ROS_DEBUG_STREAM(
      "rel. angle to goal: " << angle_diff
      << " , distance to goal: " << delta.length()
    );

    if (delta.length() < LOCATION_EPSILON) {
      // we've arrived at the next nav point

      if (done()) {
        // this was the last navigation point, we are done! wait for more.
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

  if (active) {
    nextGoal_ = nextPoseStamped_->pose.position;
    nextPoseStamped_++;
  }
  ROS_INFO_STREAM("next: " << nextGoal_.x << ", " << nextGoal_.y);
}



// ---------------------------------------------------------

void quit(int sig) {
  ROS_INFO("quit");
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "turtle_draw");
  signal(SIGINT, quit);

  TurtleDraw turtleDraw;
  ros::spin();

  return(0);
}
