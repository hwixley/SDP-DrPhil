
# Setting up the catkin workspace

## Requirements

- ROS-noetic
- Python 3.7+ (If you're using pyenv, run pyenv local system, i.e. make sure you're using system 
- Python OpenCV 2
- Numpy 
python when running commands inside the ros-workspace directory)

The process is not *that* painful:


1. `cd ros-workspace`
2. `rosdep install --from-paths src --ignore-src -r -y` - this will install all missing packages 
3. `catkin_make`
4. `source devel/setup.sh`

this "activates the workspace" to check if it worked run: `echo $ROS_PACKAGE_PATH`
you should see the following within the output:
`<path to this repo>/SDP-DrPhil/ros-workspace/src`

If you don't want to run this everytime you start a new shell you can add 

`<path to this repo>/SDP-DrPhil/ros-workspace/devel/setup.sh` to your ~/.bashrc file

On Ubuntu, and the respective files for other distros

5. To check if this works run `roscd dr-phil`, if you see no errors, you've successfully created a custom ROS workspace and installed the dr-phil ROS package.


## Starting a webots ROS controller 

For some reason I cannot get ros controllers to work without special launch files, so in order to start the simulation.wbt world simulation + launch ROS controller nodes do:

1. activate the workspace if you haven't already
2. start the roscore process with `roscore`
3. roslaunch dr-phil simulation.launch

This will run webots and start up all the controller nodes, if any errors to do with <extern> appear, simply restart the simulation which will launch the controller again. 
