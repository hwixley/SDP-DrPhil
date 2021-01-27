
# Setting up the catkin workspace

## Requirements

-ROS-noetic
-Python 3.7+ (If you're using pyenv, run pyenv local system, i.e. make sure you're using system python when running commands inside the ros-workspace directory)

The process is not *that* painful:


1. `cd ros-workspace`
2. `rosdep install --from-paths src --ignore-src -r -y` - this will install all missing packages 
3. `catkin_make`
4. `source devel.sh`

this "activates the workspace" to check if it worked run: `echo $ROS_PACKAGE_PATH`
you should see the following within the output:
`<path to this repo>/SDP-DrPhil/ros-workspace/src`

If you don't want to run this everytime you start a new shell you can add 

`<path to this repo>/SDP-DrPhil/ros-workspace/devel/setup.sh` to your ~/.bashrc file

On Ubuntu, and the respective files for other distros

5. To check if this works run `roscd dr-phil`, if you see no errors, you've successfully created a custom ROS workspace and installed the dr-phil ROS package.
