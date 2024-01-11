# =============================================================================
#   C O P Y R I G H T
# _____________________________________________/\\\\\\\\\\\_____/\\\\\\\\\_____/\\\\\\\\\\\\____
#   Copyright (c) 2021 by Robert Bosch GmbH.  _\/////\\\///____/\\\\\\\\\\\\\__\/\\\////////\\\__
#   All rights reserved.                       _____\/\\\______/\\\/////////\\\_\/\\\______\//\\\_
#                                               _____\/\\\_____\/\\\_______\/\\\_\/\\\_______\/\\\_
#   This file is property of Robert Bosch GmbH.  _____\/\\\_____\/\\\\\\\\\\\\\\\_\/\\\_______\/\\\_
#   Any unauthorized copy or use or distribution  _____\/\\\_____\/\\\/////////\\\_\/\\\_______\/\\\_
#   is an offensive act against international law  _____\/\\\_____\/\\\_______\/\\\_\/\\\_______/\\\__
#   and may be prosecuted under federal law.        __/\\\\\\\\\\\_\/\\\_______\/\\\_\/\\\\\\\\\\\\/___
#   Its content is company confidential.             _\///////////__\///________\///__\////////////_____
# _______________________________________________________________________________________________________
#   P R O J E C T   I N F O R M A T I O N
# -----------------------------------------------------------------------------
#   IAD - Infrastructure-based Autonomous Driving (CR/RIX)
# =============================================================================
#
#
#  Created on: March 23, 2021
#      Author: zns5sgh
#     Contact: fixed-term.Simon.Zhang@cn.bosch.com
#

from setuptools import setup
import os
from glob import glob

package_name = 'readcan'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('dbc/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zns5sgh',
    maintainer_email='fixed-term.Simon.ZHANG@cn.bosch.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'readcan = readcan.DecodeMsg_foxy:main',
        ],
    },
)
