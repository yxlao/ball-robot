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
        rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0254 left:=/left_cam/image_raw left_camera:=/left_cam right:=/right_cam/image_raw right_camera:=/right_cam --approximate=0.05s

        # for single camera
        rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0254 image:=/left_cam/image_raw camera:=/left_cam

## Conventions
1. To add a new repository, use `git add submodule`, for example camera calibration

        git submodule add  https://github.com/ros-perception/image_pipeline.git
