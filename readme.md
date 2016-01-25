Assume your `workspace` is `~/robot`
1. Backup and remove your `~/robot/src` folder
2. Clone to `~/robot/src`
```sh
# replace with your own git address
# must clone to the src folder
git clone git@bitbucket.org:ucsd_team_one/robot-central.git ~/robot/src
```
3. Make multiple times (to get rid of make errors)
In your workspace `~/robot`
```sh
catkin_make
catkin_make
catkin_make
```
