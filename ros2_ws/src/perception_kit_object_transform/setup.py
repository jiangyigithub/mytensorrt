# Copyright (c) 2009, 2018 Robert Bosch GmbH and its subsidiaries. This program and the accompanying materials are made
# available under the terms of the Bosch Internal Open Source License v4 which accompanies this distribution, and is
# available at http://bios.intranet.bosch.com/bioslv4.txt

## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['perception_kit_object_transform'],
    package_dir={'':'python'})

setup(**setup_args)
