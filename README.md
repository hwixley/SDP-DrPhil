
# Setting up the catkin workspace

## Requirements

-ROS-noetic
-Python 3.7+ (If you're using pyenv, run pyenv local system, i.e. make sure you're using system python when running commands inside the ros-workspace directory)

## Installation 

The process is not *that* painful:


1. `cd ros-workspace`
2. `rosdep install --from-paths src --ignore-src -r -y` - this will install all missing packages (most, if it fails to install some you'll have to install manually)
3. `catkin_make`
4. `source devel/setup.sh`

this "activates the workspace" to check if it worked run: `echo $ROS_PACKAGE_PATH`
you should see the following within the output:
`<path to this repo>/SDP-DrPhil/ros-workspace/src`

If you don't want to run this everytime you start a new shell you can add 

`<path to this repo>/SDP-DrPhil/ros-workspace/devel/setup.sh` to your ~/.bashrc file

On Ubuntu, and the respective files for other distros

5. To check if this works run `roscd dr-phil`, if you see no errors, you've successfully created a custom ROS workspace and installed the dr-phil ROS package.

## Starting gazebo simulation

1. activate the workspace if you haven't already
2. `roscore`
3. roslaunch dr-phil simulation.launch

### Interfacing with the robot

see docs https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation

- Teleoperation (keyboard control)
    - `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`
- See all available topics
    - `rostopic list`
- Publish to a topic
    - e.g. `rostopic pub /cmd_vel geometry_msgs/Twist -- '[0.1,0,0]' '[0,0,0.1]' `
- Information on a topic
    - e.g. `rostopic info /cmd_vel`

### Visualising sensor data 

Run `rviz`
You can visualise the data by loading the config in `dr-phil/rviz/model.rviz` from rviz when running the gazebo simulation

### using SLAM and other launch files

if you want to follow the instructions provided on the docs, that's fine
however the launch files in their packages point to the model within the actual turtlebot packages, so in order to use those launchfiles correctly you must do something similar to what was done in the simulation.launch file ,i.e. look into the package 

say `roscd turtlebot3_gazebo/launch` check out the launch files and traverse the tree to see what they're actually launching (most of the time it's plugins from other packages with some arguments) we need our own versions of those launch files which point to our models in /urdf 

slam functionality is actually provided via external node, which simply must be launched with the appropriate arguments, again have a look at the launch files


### simple commands

see: http://www.inf.ed.ac.uk/teaching/courses/sdp/SDP2020/turtlebot3_docs.pdf
for usage of `/cmd_vel` and `/joint_trajectory_point` for manually controlling the bot and arm


### Decreasing performance issues

If your simulation is very slow, and you don't need all the features of the simulation (just the robot) then change the world to `field.world` like so:

`roslaunch dr-phil simulation.launch world:=field`

## Starting webots simulation 

For some reason I cannot get ros controllers to work without special launch files, so in order to start the simulation.wbt world simulation + launch ROS controller nodes do:

1. activate the workspace if you haven't already
2. start the roscore process with `roscore`
3. roslaunch dr-phil simulation-webots.launch

This will run webots and start up all the controller nodes, if any errors to do with <extern> appear, simply restart the simulation which will launch the controller again. 

## Behaviours

we're using py_trees to model complex behaviours, for documentation see here:

https://py-trees.readthedocs.io/en/release-0.7.x/