# Robot Design Overview

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

## 5 Checkpoints from Week 6

- Week 6
    - All sensors working.
    - Attach potentiometers to the relevant joints of the arm and wire them.
    - Be able to at least approximate the angle of each of the joints of the arm.
    - Be able to pass all sensor information through to the odroid
    - Be able to move arm to a certain position.
    - Be able to calculate the angle each joint
    - Be able to accurately approximate the location in space of each joint,
      at least relative to the previous joint.
    - Wire potentiometers up to Mbed.
    - Set up and process information from a third camera.
    - Ultimate goal: be able to pick up a stationary ball.
- Week 7: Robot control
    - Visualize all arm joints as transforms in rviz
    - Obstacle avoidance: be able to navigate around a certain object in the way.
    - Response to bump sensors, cliff sensors, etc.
    - Bump sensor -> back up, turn, try to drive around it, maybe rotate to re-
      find ball
    - Cliff sensor -> back up and turn, move over a bit and then try to resume
      path
    - Proximity sensor -> look for ball?
    - Driving toward a ball and stop in front of it.
    - Drive to a set coordinate
    - Be able to turn until a ball is located and then stop turning.
    - Be able to verify that the ball is still there when we're driving to it
Week 8:
    - Computer vision -> ignore big obstacles of the same color but different
      size / shape
    - Consistently track the same ball when multiple balls are in the frame
    - Reset state machine after dropping off ball
    - Robot control-what to do with respect to other robots?
    - Final arm mount
    - Be able to find and go to goal / bucket
    - Raise ball to a certain height in front of the robot and then drop it (to
      simulate putting it into a basket)
    - By this time we want to have all of the components working so we can piece
      it all together.
- Week 9:
    - Detect ball vs paper
    - Last-minute debugging
