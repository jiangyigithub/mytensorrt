# Radar GEN5

[![License: BIOSL v4](http://bios.intranet.bosch.com/bioslv4-badge.svg)](#license)

This repository contains packages, which allow to decode, process and to publish information gathered from radar gen5 sensors.

Naming conventions for packages:

- Packages with the ‘_ros’ postfix depend on ros libraries such as roscpp or rosbag. These packages also include functionality to start a ros node to process and publish data.
- Packages without the ‘_ros’ postfix do not depend on ros libraries but still are compiled with catkin as a ros package.

Raw sensor data will be read and published within the *radar_manager_ros* package.

**Note:** if you clone the repo as a submodule in your repository all other users of your repository have to run `git submodule init` and `git submodule update`.

## Current limitations (after migration to ROS2)

- Currently only one radar sensor supported due to some differences in parameter handling, however solution concept already exists and will be implemented upon demand (thanks to ROS2 foxy)
- Due to strict ROS2 naming conventions, some messages had to be renamed, while leaving the content unchanged. Thus, old data recordings from ROS1 might not be useable with the ROS-bridge, unless s.o. finds a nice solution for that.
- RadarRosTimeConverterInterface not migrated to ROS2, as mostly the trivial implementation was used anyway
- yaml config files for manager and sensor devices are combined now (and I do not see why this shouldnt stay)
- Check for Radar device type device_type_names was removed, as it anyway had been declared deprecated earlier, and we trust our radar decoder users to know what they are doing. Decoding unsupported radar binaries will fail due to the radar_sw check anyway during radar_location_decoder.

## ROS2 first steps

Example scripts and config files are checked-in and can be launched with

```sh
ros2 launch radar_manager_ros radar_manager_top95_lb.launch.py
ros2 launch radar_locations_decoder_gen5_ros radar_locations_decoder_top95_lb.launch.py
ros2 launch radar_locations_to_pointcloud_converter_ros radar_locations_to_pointcloud_converter_top95_lb.launch.py
```

## Getting Started

To clone this project as a standalone repository:

```sh
git clone ssh://git@sourcecode.socialcoding.bosch.com:7999/rs/radar_gen5.git
```

To clone this project as a git submodule:

```sh
cd your_repo
git submodule add ssh://git@sourcecode.socialcoding.bosch.com:7999/rs/radar_gen5.git
```

### Catchkin

Unit tests in this repo are written with catchkin - a software package that integrates the Catch C++ testing framework into Catkin.

To clone the repository run:

```sh
git clone ssh://git@sourcecode.socialcoding.bosch.com:7999/rs/catchkin.git
```

Further information on catchkin:
[https://inside-docupedia.bosch.com/confluence/display/MFAD/Unit+Testing+with+Catchkin](https://inside-docupedia.bosch.com/confluence/display/MFAD/Unit+Testing+with+Catchkin)

### VFC

This repository depends on [vfc package]( https://sourcecode.socialcoding.bosch.com/projects/VFC/repos/vfc/browse). Unfortunately, there are multiple forks of this package used by different teams at Bosch.
The radar_gen5 repo needs *vfc::float16_storage_t* type to be able to decode locations and objects. This type is currently not available in the official repo.
The possible solution is to clone [maplo-vfc]( https://sourcecode.socialcoding.bosch.com/projects/RS/repos/maploc-vfc/browse) which already has a ros wrapper so that it is possible to compile it with catkin.

```sh
git clone ssh://git@sourcecode.socialcoding.bosch.com:7999/rs/maploc-vfc.git
```

## Used 3rd party Licenses
Software | License
---------|--------
[ROS](https://www.ros.org/) | [BSD 3-clause](https://opensource.org/licenses/BSD-3-Clause)
[Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) | [MPL2](http://www.mozilla.org/MPL/2.0) / [LGPLv3](http://www.gnu.de/documents/lgpl-3.0.de.html)
[yaml-cpp](https://github.com/jbeder/yaml-cpp) | [MIT](https://opensource.org/licenses/MIT)
[PCL](https://pointcluds.org) | [BSD 3-clause](https://opensource.org/licenses/BSD-3-Clause)
[CPPUnit](https://freedesktop.org/wiki/Software/cppunit/) in [vfc package]( https://sourcecode.socialcoding.bosch.com/projects/VFC/repos/vfc/browse) | [LGPLv2](https://sourceforge.net/directory/license:lgpl/)

**Note:** This list may be incomplete or out of date.

## Building

```sh
colcon build
```

## Contribution Guidelines

Please pay attention to the [Coding Conventions](https://inside-docupedia.bosch.com/confluence/display/MFAD/Coding+Conventions) when adding new packages or code to the repo.

## About

### Maintainers

- [Ulrich Michael (CR/AEV4)](https://connect.bosch.com/profiles/html/profileView.do?uid=ULI8FE) (Michael.Ulrich2@de.bosch.com)
- [Mate Zoller (CC-AD/EYF2-Bp)](https://connect.bosch.com/profiles/html/profileView.do?uid=ZOL2BP) (Mate.Zoller@hu.bosch.com)
- [Tobias Knispel (CC-AD/EYF5)](https://connect.bosch.com/profiles/html/profileView.do?uid=KNT7ABT) (Tobias.Knispel@de.bosch.com)
- [Artur Quiring (CC-AD/EPM3)](https://connect.bosch.com/profiles/html/profileView.do?uid=QUA2HI) (Artur.Quiring@de.bosch.com)
- [Vladimir Belyaev (CC-AD/EPM3)](https://connect.bosch.com/profiles/html/profileView.do?uid=BEV4ABT) (Vladimir.Belyaev@de.bosch.com)
- [Joachim Börger (CR/AAS2)](https://connect.bosch.com/profiles/html/profileView.do?uid=BOJ1YH) (Joachim.Boerger@de.bosch.com)

### Repository Owner & Access Rights

- [Valentin Frommherz (CC-AD/EPM1)](https://connect.bosch.com/profiles/html/profileView.do?uid=FVA2ABT) (Valentin.Frommherz@de.bosch.com)

