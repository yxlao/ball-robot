# Robot Design Overview
**Link to the videos**

**Robot's capabilities**

- Detect orange and grenn balls and select closest target
- Distinguish balls from color papers
- Drive towards to the ball
- Picku up the ball with robotic arm
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
- TP-LINK rounter * 1
- Robotic arm * 1 (with 4 motors)
- Laptop * 1 (for launching and monitoring)
- iRobot cleaner * 1
- Battery packs * 3
- USB camera * 3
- Plastic platform * 1

### 1.2 Hardware architecture

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
        - Ball dtection node
        - Voltage receiver node
        - Ultrasonic distance sensor node

## 2 Software
### 2.1 State machine
### 2.2 Ball detection
### 2.3 Arm control
## 3 Code structure
## 4 Notes on launching
## 5 Time-line and checkpoints