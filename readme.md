## Running
Assume your `workspace` is `~/robot`.

1. Backup and remove your `~/robot/src` folder.
2. Clone to `~/robot/src`.

        # replace with your own git address, must clone to the src folder
        git clone git@bitbucket.org:ucsd_team_one/robot-central.git ~/robot/src

3. Make multiple times (to get rid of make errors).

        # In your workspace `~/robot`
        catkin_make
        catkin_make

## Conventions
1. To add a new repository, use `git add submodule`, for example camera calibration

        git submodule add  https://github.com/ros-perception/image_pipeline.git
