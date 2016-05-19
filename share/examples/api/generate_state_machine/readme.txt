# for further description see:
https://rmintra01.robotic.dlr.de/wiki/RAFCON/Tutorials

# run generation in one shell with previous source of ROS-Indigo and RAFCON environment
$ source /volume/USERSTORE/lehn_pt/ros/indigo/setup.bash
$ rmpm_do env rafcon > /tmp/rafcon.env
$ source /tmp/rafcon.env
$ python basic_turtle_state_machine.py

# to see what happens run additional in separate shells
$ source /volume/USERSTORE/lehn_pt/ros/indigo/setup.bash
$ roscore

$ source /volume/USERSTORE/lehn_pt/ros/indigo/setup.bash
$ rosrun turtlesim turtlesim_node
