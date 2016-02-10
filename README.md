# turtles_draw

## Get the Code:

```sh
git clone https://github.com/chfritz/turtles_draw.git
```

## Dependencies and Compile ROS Node

Run the `setup.sh` script to install all dependencies, incl. meteor.

## Running on Ubuntu 14.04

### ROS and Turtlesim

In separate terminals, start `roscore`, turtlesim (`rosrun turtlesim
turtlesim_node`), and rosbridge (`roslaunch rosbridge_server
rosbridge_websocket.launch`).

Note: the current implementation assumes that there is no delay coming from the
rendering of the turtlesim window. This means that if you are running turtlesim
remotely, you won't be able to use the GUI because X-forwarding would be too
slow. In order to still try it out you can create a dummy, headless X server:

```sh
sudo apt-get install xvfb
Xvfb -shmem -screen 0 1280x1024x24
DISPLAY=:0 rosrun turtlesim turtlesim_node
```

Of course, now you'll *have* to use the meteor app to see what's happening.


### Meteor App:

in `turtles_draw/app` run:

```sh
meteor
```

This will at first take a while because it will install roslibjs and compile its
native code (this is why we needed cairo, which gets installed by the setup.sh
script). If anything fails here, see
https://github.com/RobotWebTools/roslibjs/pull/104 and makre sure all
depedencies are installed.

Once meteor has started (you'll see `=> App running at: http://localhost:3000/`),
launch a browser at `localhost:3000`. You should see a black square.


### ROS Node:

in `turtles_draw/ros` run (this was already compiled in the setup.sh script):

```sh
. devel/setup.bash
rosrun turtle_draw turtle_draw_node shape1.json
```

You will see lines being drawn with small waiting periods in between. Eventually these lines will close to a star. You can also try the other shapes `shape2.json`, and `shape3-norte_star.json`.



## Running on OSX
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
