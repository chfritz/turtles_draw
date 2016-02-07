# turtles_draw


## ROS Node


To compile: in `ros/` run

```sh
catkin_make
. devel/setup.bash
```

To run: start `roscore` and turtlesim (`rosrun turtlesim turtlesim_node`)
and then, in `ros/`, run

```sh
rosrun turtle_draw turtle_draw_node shape1.json
```


## Meteor app

### Dependencies:

https://github.com/RobotWebTools/roslibjs/pull/104

#### on OSX:
The app installs it's own npm dependencies, but on OSX El Capitan, I saw an
error related to `jpeglib.h` when installing npm dependencies for roslibjs (used
by chfritz:meteor-ros). I got past this by running:

```sh
xcode-select --install
```
as described here https://github.com/Automattic/node-canvas/issues/649.


### install rosbridge

```sh
sudo apt-get install ros-jade-rosbridge-suite
```

on OSX you'll need to compile that from source (https://github.com/RobotWebTools/rosbridge_suite/issues/198#issuecomment-159776996):

```sh
# required on el capitan to find openssl headers (https://github.com/phusion/passenger/issues/1630):
ln -s /usr/local/Cellar/openssl/1.0.2e_1/include/openssl/ /usr/local/include/

# install dependency
git clone git@github.com:WPI-RAIL/rosauth.git
mv rosauth src
catkin_make
. devel/setup.bash
sudo pip install bson
sudo pip install twisted
sudo pip install pymongo
```

### launch rosbridge

```sh
roslaunch rosbridge_server rosbridge_websocket.launch
```

### launch the app

```
meteor
```

then launch a browser at `localhost:3000`
