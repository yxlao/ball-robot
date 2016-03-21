# Robot Design Overview

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
    - Connected to bottom USB **camera**, ball detection for bottom camera
    - Connected to middle USB **camera**, ball detection for middle camera
- Raspberry Pi 2
    - Connected to **mbed**, reading values for 4 **potentiometers**
    - Connected to **Arduino**
        - Connected to 4 motors via **Arduino Motor Shield**
        - Connected to 2 **ultrasonic distance sensors**
    - Connected to arm USB **camera**, ball detection for arm camera
    - Runs ROS Master node

## 2 Software
### 2.1 State machine
### 2.2 Ball detection
### 2.3 Arm control
## 3 Code structure
## 4 Notes on launching
## 5 Time-line and checkpoints