# Ball Robot
![robot](https://cloud.githubusercontent.com/assets/1501945/14213494/1a2451d0-f7ed-11e5-91e6-54b4bf534c53.jpg)

A robot that can drive to the ball, pick it up and put it into the basket

## Robot's capabilities

- Detect orange and green balls and select closest target
- Distinguish balls from color papers
- Drive towards to the ball
- Pick up the ball with robotic arm
- Front and back avoid obstacle
- Detect bucket
- Drive towards the bucket
- Drop the ball into the bucket with robotic arm

## 1 Hardware

### 1.1 List of components

- Raspberry Pi 2 Model B * 2
- Arduino * 1
- Arduino Motor Shield * 1
- Mbed * 1
- Potentiometer * 3
- TP-LINK router * 1
- Robotic arm * 1 (with 4 motors controlled)
- Laptop * 1 (for launching and monitoring)
- iRobot cleaner * 1
- Battery packs * 3
- USB camera * 3
- Plastic platform * 1

### 1.2 Hardware hierarchy

Here, we describe the responsibility of each hardware components

- Raspberry Pi 1
    - Connected to bottom USB **camera** and ball detection for bottom camera
    - Connected to middle USB **camera** and ball detection for middle camera
    - Connected to **iRobot** via USB and controls the iRobot
    - Runs
        - 2 camera nodes
        - Ball detection node
        - iRobot control node (the main state machine)
- Raspberry Pi 2
    - Connected to **mbed** via USB and read values for 4 **potentiometers**
    - Connected to **Arduino** via USB
        - Connected to 4 motors via **Arduino Motor Shield**
        - Connected to 2 **ultrasonic distance sensors**
    - Connected to arm USB **camera**, ball detection for arm camera
    - Runs
        - ROS Master node
        - Motor control node
        - Ball detection node
        - Voltage receiver node
        - Ultrasonic distance sensor node

## 2 Software

### 2.1 State machine
We designed hierarchal state machines, where each main states can have multiple
sub states (for example the `avoid` and `explore` states). The robot the
internal states and external sensor inputs determines the robots action, and
ultimately changes the internal states. For more details, please take a look
at `create_driver/creat_state_machine.py`.

- `find_ball`
- `drive`
- `turn_right`
- `turn_left`
- `back`
- `avoid`
    - `stop`
    - `back_up`
    - `turn_left`
    - `turn_right`
    - `drive`
- `explore`
    - `explore_drive`
    - `explore_turn`
- `fine_position`
- `pick_up`
- `find_bucket`
- `stop`

### 2.2 Ball detection
- **Main idea**
  The main methods we used is HSV color thresholding.In order to determine the
  most suitable thresholds, we implemented a GUI program that allow us to pick
  the region in the image and set the thresholds according to the color in that
  area. We also use OpenCV's `medianBlur`, `erode` and `dilate` functions to
  produce cleaner detection results. The robot will aim for the largest
  detected ball.
- **Distinguish balls against color paper**
  For each detected color blobs, we can obtain the contour information from
  OpenCV. Then we get the following features about the contour:
    - roundness factor: the ratio of contour area and the bounding circle area)
    - squareness factor: the ratio of contour area and the bounding rectangle
                         area
    - centroid height
    - contour area
  Then collect a bunch of data regarding the ball and paper, and trained a
  a Gradient Boosted Tree classifier to classify the feature. As long as the
  data is collected on the actual field, the classification accuracy can be
  about 95%.

### 2.3 Arm control
We control 4 motors in the arm. Each of the motor (except the claw motor) has a
potentiometer attached to it. The arm is controlled via feedback mechanism with
the potentiometer. We wrote several macros (e.g. pick ball, raise arm, put
in buckets) to perform the tasks accordingly.

## 3 Code structure

- `arm_control`: Arm controller, mbed angle reader
- `ball_detect`: Ball detection for the 3 cameras
- `create_driver`: iRobot driver node and the main state machine

## 4 Notes on launching

Bottom Raspberry

    # 1 in fine_arm_control branch
    roslaunch arm_control start_arm_control.launch
    # 2 make sure ROS_IP
    export ROS_IP=192.***.***.***
    echo $ROS_IP

Top Raspberry

    # 1 recheck bashrc
    ROS_IP set to itself
    ROS_MASTER_URI=http://the_other_ip:11311
    source ~/robot/devel/setup.bash
    # 2 without running anything, try
    rostopic list
    # 3 push button
    # 4 launch main state machine
    roslaunch create_driver vision_integration_start.launch
    # 5 if gscam related problem, unplug and plug in the camera again
    #   stero_left should be bottom camera, when unplug, plug in bottom first


Manual Commands for Debug

    rosrun create_driver keyboard_state_changer.py
    W: forward
    A: left
    D: right
    X: back
    S: stop

    rosrun arm_control basic_pickup_macro.py
    corresponding to motor
    W E R T O
    S D F G C
    X

## 5 Misc

### Backup and restore sd card

    # to backup
    $ sudo dd bs=4M if=/dev/sdb | gzip > lubuntu_feb_18.img.gz
    $ watch -n5 sudo pkill -usr1 dd
    # to restore
    gunzip --stdout lubuntu_feb_18.img.gz | sudo dd bs=4M of=/dev/sdb
