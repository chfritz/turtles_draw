screen -d -m -S core roscore
screen -d -m -S sim rosrun turtlesim turtlesim_node
# rosrun turtlesim turtle_teleop_key
. ~/local/rosbridge_suite/devel/setup.bash
. ~/local/rosauth/devel/setup.bash
screen -d -m -S bridge roslaunch rosbridge_server rosbridge_websocket.launch
screen -d -m -S topic rostopic echo /turtle1/pose

screen -d -m -S app (cd app && meteor)
. devel/setup.bash
rosrun turtle_draw turtle_draw_node shape3-norte_star.json 
