#!/bin/bash

echo
echo "install ros and ros packages we need"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install git g++ ros-jade-ros-base ros-jade-rosbridge-server ros-jade-turtlesim libcairo2-dev libjpeg8-dev libpango1.0-dev libgif-dev build-essential
sudo rosdep init
rosdep update
echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo
echo "install meteor"
curl https://install.meteor.com/ | sh

echo
echo "get code and compile"
cd ros/
catkin_make
. devel/setup.bash

echo "we are done setting up, now continue with the Readme.md"
