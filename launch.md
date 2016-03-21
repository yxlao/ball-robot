# Notes on launching the whole system

## Trun on bottom raspberry
1. In fine_arm_control branch
    ```
    source setup.bash
    roslaunch arm_control start_arm_control.launch
    ```

2. make sure ROS_IP=the self ip
    ```
    export ROS_IP=192.***.***.***
    echo $ROS_IP
    ```

## Turn on top raspberry
1. edit bashrc:
    * ROS_IP of it self
    * ROS_MASTER_URI=http://the_other_ip:11311
    * source ~/robot/devel/setup.bash
2. without running anything, run following, should show joints/angle etc
    ```
    rostopic list
    ```

##
###
1. push button, make sure to lauch i robot before running
2. roslaunch create_driver vision_integration_start.launch
3. if gscam related problem, unplug and plug in the camera again

###
Manual control
```
rosrun create_driver keyboard_state_changer.py
W: forward
A: left
D: right
X: back
S: stop
```

##
Macro to pick up the ball now
```
rosrun arm_control basic_pickup_macro.py
corresponding to motor
W E R T O
S D F G C
X
```
pick up macro: 1
stop pick up macro: 2

##
top pi,
stero_left should be bottom camera
when unplug, plug in bottom first

