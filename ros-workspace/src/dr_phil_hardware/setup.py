## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

# see instructions on creating python modules:
# http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile

from distutils.core import setup 
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['dr_phil_hardware'],
    package_dir={'':'src'}
)


setup(**setup_args)