```
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
```
# TCP2ROS package
This is a ROS2 version of the tool for resolving tcp bags from the CANFD-TCP Converter Box. Detailed description please take the master branch as a reference.

# Usage
If you use ZLG Box:
`ros2 run tcp2ros tcp2ros ZLG`

Or
put this into launch file as in launch/tcp2ros_launch.py:
`ros2 launch tcp2ros_launch.py`

## Library:
[`ROS`](https://www.ros.org/)


## About

|                    |         |
| -------------------|---------|
| Maintainers        |<a href="mailto:fixed-term.Simon.Zhang@cn.bosch.com">Simon Zhang</a> & <a href="mailto:external.xinrun.li2@cn.bosch.com">Xinrun LI</a> & <a href="mailto:Jinyao.Liang@cn.bosch.com">Jingyao Liang</a>|
| Used Encryption    | No custom encryption is used (see BIOS Repository Policy §4.a) |
| 3rd party licenses | None    |

<!-- ### License

[![License: BIOSL v4](http://bios.intranet.bosch.com/bioslv4-badge.svg)](#license)

> Copyright (c) 2009, 2018 Robert Bosch GmbH and its subsidiaries.
> This program and the accompanying materials are made available under
> the terms of the Bosch Internal Open Source License v4
> which accompanies this distribution, and is available at
> http://bios.intranet.bosch.com/bioslv4.txt -->


