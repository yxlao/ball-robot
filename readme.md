# Robot Design Overview
**Link to the videos**

**Robot's capabilities**

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
- Potentiometer * 4
- TP-LINK router * 1
- Robotic arm * 1 (with 4 motors)
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
ultimately changes the internal states.

#### 2.1.1 List of states

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

### 2.3 Arm control

## 3 Code structure

## 4 Notes on launching

## 5 Time-line and checkpoints
