# Perception Kit Messages  <!-- omit in toc -->

[![License: BIOSL v4](http://bios.intranet.bosch.com/bioslv4-badge.svg)](https://inside-docupedia.bosch.com/confluence/display/SCOS/BIOS+License+V4)
[![Build Status](http://asy-ci-ape.rng.de.bosch.com:8080/buildStatus/icon?job=perception_kit_msgs%2Fmaster)](http://asy-ci-ape.rng.de.bosch.com:8080/job/perception_kit_msgs/job/master/)

This repository contains ROS messages for perception kit components as follows:

### Dynamic Object List

| Message            | Description                                                                    |
| -------------------|--------------------------------------------------------------------------------|
| Objects.msg        | An array of Object instances (dynamic object list)                             |
| Object.msg         | Dynamic 3D object with properties like shape (cuboid), object type or velocity |
| Classification.msg | Object type and classification confidence used within Object instances         |
| Attribute.msg      | Extra information attached to Object instances                                 |

### Dynamic Occupancy Grid

| Message                  | Description                                                              |
| -------------------------|--------------------------------------------------------------------------|
| DynamicOccupancyGrid.msg | Extends `nav_msgs/OccupancyGrid` with classification of occupied cells   |

### Odometry and Ego-motion

| Message            | Description                                                                    |
| -------------------|--------------------------------------------------------------------------------|
| Motion.msg         | Extends `nav_msgs/Odometry.msg` by acceleration and curvature                  |

## Getting Started

Clone this repository in the ```src/``` folder of a catkin workspace.

```bash
cd <your_catkin_workspace>/src
git clone ssh://git@sourcecode.socialcoding.bosch.com:7999/perception_kit/perception_kit_msgs.git
```

You can also use a subdirectory like ```src/messages``` or ```src/interfaces``` if your prefer.

## Building and Testing

Build and use the message like any other ROS message definitions.

```bash
catkin build perception_kit_msgs
source <your_catkin_workspace>/devel.bash
rosmsg info perception_kit_msgs/Object
rosmsg info perception_kit_msgs/Objects
```

## About

For contribution guidelines and feedback please refer to [perception_kit](https://sourcecode.socialcoding.bosch.com/projects/PERCEPTION_KIT/repos/0_perception_kit/browse/README.md).

|                    |         |
| -------------------|---------|
| Maintainers        | <a href="mailto:dennis.nienhueser@de.bosch.com">Dennis Nienhüser</a>, <a href="mailto:tobias.baer3@de.bosch.com">Tobias Bär</a> |
| Used Encryption    | No custom encryption is used (see BIOS Repository Policy §4.a) |
| 3rd party licenses | None    |

### License

[![License: BIOSL v4](http://bios.intranet.bosch.com/bioslv4-badge.svg)](#license)

> Copyright (c) 2009, 2018 Robert Bosch GmbH and its subsidiaries.
> This program and the accompanying materials are made available under
> the terms of the Bosch Internal Open Source License v4
> which accompanies this distribution, and is available at
> http://bios.intranet.bosch.com/bioslv4.txt
