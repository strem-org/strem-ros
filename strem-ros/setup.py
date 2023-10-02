#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Use the ROS package metadata from `package.xml` to generate the
# necesssary arguments to install the scripts and Python packages.
#
# This simply sets things up. However, the `setup` function must still be
# called with these arguments expanded.
args = generate_distutils_setup(
    packages=["stremros"],
    scripts=[
        "nodes/annotations",
        "scripts/perception-sim"
    ],
    package_dir={"": "src"},
)

# Call the `setup` function to install the package
# and scripts, appropriately.
setup(**args)
