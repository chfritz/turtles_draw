rosrun turtlesim turtlesim_node
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install ros-jade-ros-base
sudo apt-get install ros-jade-rosbridge-server
sudo rosdep init
rosdep update
echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install git
sudo apt-get install g++
sudo apt-get install ros-jade-turtlesim
git clone https://github.com/chfritz/turtles_draw.git
curl https://install.meteor.com/ | sh

cd turtles_draw/
cd ros/
catkin_make
. devel/setup.bash 
# cd ..
# ./run.sh 
# rosrun turtle_draw turtle_draw_node shape3-norte_star.json
