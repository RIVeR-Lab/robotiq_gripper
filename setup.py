## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
## use this script to install custom ros python module

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['robotiq_gripper'],
    package_dir={'': 'src'})

setup(**setup_args)