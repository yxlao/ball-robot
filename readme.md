## Running
Assume your `workspace` is `~/robot`.

1. Backup and remove your `~/robot/src` folder.
2. Clone to `~/robot/src`.

        # replace with your own git address, must clone to the src folder
        git clone git@bitbucket.org:ucsd_team_one/robot-central.git ~/robot/src
        git submodule init
        git submodule update

3. Make multiple times (to get rid of make errors).

        # In your workspace `~/robot`
        catkin_make
        catkin_make


## Calibration

        # for stero camera
        rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0254 left:=/left_cam/image_raw left_camera:=/left_cam right:=/right_cam/image_raw right_camera:=/right_cam --approximate=0.1

        # for single camera
        rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0254 image:=/left_cam/image_raw camera:=/left_cam

## Conventions
1. To add a new repository, use `git add submodule`, for example camera calibration

        git submodule add  https://github.com/ros-perception/image_pipeline.git

## Disparity Map
1. The launch file should set up the cameras to start recording automatically. Then, to calculate the disparity map, run

       ROS_NAMESCE=/stereo rosrun stereo_image_proc stereo_image_proc

If everything works correctly, it should broadcast to the /stereo/disparity topic. Also, refreshing the image view should show a lot more topics to view images from.

2). Then, to view the left and right rectified images and the disparity map run

rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color _approximate_sync:=True


## 1 Devices used

* Robotics arm

* Odroid XU4

* Mbed

* Arduino

* Motor shield

* Variable resister



## 2 Design choices: communications

* Variable resister -> Mbed: Analog in

* Mbed <-> Odroid: Via USB serial.  we used UART pins on odroid before (which was presented in the video), but it was less stable then a USB. We added a USB hub and make sure its compatibility and power usage.

* Odroid <-> Arduino: Via USB cable.

* Arduino -> Motor shield: I2C, soldered pin connection on motor shield.



## 3 Implementation steps:

1. Setup robotic arm, mount variable resister on robotic arm

2. Measure voltage using mbed

3. Control motor via arduino

4. Writeup ROS node for motor control

5. System intergration



Assignment Submission videos:



Assignment 1 https://youtu.be/UvxVTmcM6as

Assignment 2 https://youtu.be/iocWSl63kQ8