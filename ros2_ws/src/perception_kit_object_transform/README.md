<!---

	Copyright (c) 2009, 2018 Robert Bosch GmbH and its subsidiaries.
	This program and the accompanying materials are made available under
	the terms of the Bosch Internal Open Source License v4
	which accompanies this distribution, and is available at
	http://bios.intranet.bosch.com/bioslv4.txt

-->

# Perception Kit Object Transform  <!-- omit in toc -->

[![License: BIOSL v4](http://bios.intranet.bosch.com/bioslv4-badge.svg)](#license)

The perception kit object transform package implements the ability to transform `perception_kit_msgs::Object`s from one
ROS tf frame to another ROS tf frame **considering the motion of the frames, too**.
It transforms the pose, as well the acceleration and the velocity values including all covariances. The transform
invariant values, as for instance the id or the classification, remain untouched.

The transform functionality is available in three different manners:
 * As `nodelet`, which can be started with your ROS stack. This is handy, if for instance an object tracking component
   outputs its objects in a `odom` frame, but the further processing component needs the objects in `base_link`.
 * As `cpp-library` which you can link to your component and invoke a `transformObject` there.
 * As `python-module` which you can import into your python code and invoke a `transformObject` there.

Transforming the objects pose from one coordinate system to another can simply be obtained by using the `tf` or `tf2`
functionality coming with **ROS**. However, the `perception_kit_msgs::Object`s have motion values and the coordinate
systems move relative to each other in time, too. Hence, these motion values need to be transformed, too.

Let's have a look at a simple example (which is tested in unit test `transformObject_following_object` in
`test/unit_tests/test_perception_kit_object_transform.cpp` this repository):

* A car is driving in an east north up coordinate system with a velocity of `1 m/s` over ground in east direction.
* The observer (ego vehicle) is 2 meters behind, having the same velocity in east direction and no velocity towards
  another cardinal direction. The observers coordinate system is the `base_link` frame.
* The object tracking component is providing its objects in the east north up (`enu`) coordinate system.
* For simplicity, the `base_link` frame matches the `enu` coordinate system for this moment in time.
* The post processing unit, wants the objects to be in `base_link`.

|                | Input | Output      | Comment |
|----------------|-------|-------------|---------|
Coordinate Frame | `enu` | `base_link` | as the post processing wants the objects in `base_link`
Pose.Position    | 2,0,0 | 2,0,0       | as we said `base_link` matches `enu` atm
Motion.Velocity  | 1,0,0 | 0,0,0       | as the vehicles drive behind each other with the same velocity and direction.

For other *theoretical* examples, you can have a look at the unit tests. Feel free to implement some more!

## Table of Contents  <!-- omit in toc -->

- [Getting Started](#getting-started)
  - [Usage as nodelet](#nodelet)
  - [Usage as cpp library](#cpplib)
  - [Usage as python module](#python)
- [Limitations](#limitations)
- [Building and Testing](#building-and-testing)
- [Contribution and Feedback](#contributing)
- [About](#about)
  - [License](#license)

## Getting Started <a name="getting-started"></a>

This section should contain information on how to use the content of this
repository. If it contains SW, consider explaining how it is installed, run or integrated.

### Usage as nodelet <a name="nodelet"></a>

As mentioned, you can launch this transformation component as a nodelet in your ROS stack. An example launch file,
together with a example config file is included in the `launch` and `config` directories.

In this config file example, a `perception_msgs::Objects` list, published on the topic `/fusion/obstacles` with the
objects being in the `odom` frame is transformed into a `perception_kit_msgs::Objects` list in `base_link` frame,
published on topic `/perception_kit/fusion/objects`. The `target_frame_needs_motion` flag indicates, that the coordinate
frames are in motion to each other. And the motion section specifies, the topic where the motion 
(`perception_kit_msgs::Motion`) is published.

```
object_transform_example.launch:

input:
 frame: "odom"
 topic: "/fusion/obstacles"

output:
 frame: "base_link"
 topic: "/perception_kit/fusion/objects"
 target_frame_needs_motion: true

motion:
  topic: "/egoestimation/odometry_referenced_egomotion"
```

### Usage in your cpp code <a name="cpplib"></a>

To use the transform functionality in your cpp code, you need to
* include the `perception_kit_object_transform` package in your packages `CMakeLists.txt` and `package.xml`
* include the `perception_kit_object_transform/object_transform.h` header in your cpp code
* call the `perception_kit::object_transform::objectTransform(object, transform, velocity, acceleration)` function

For some examples, you can refer to the unit tests, included in `tests/unit_tests/`.

### Usage in your python code <a name="python"></a>

To use the transform functionality in your python code, you need to
* `import perception_kit_object_transform` in your python file
* call `perception_kit_object_transform.transformObject(object, transform, velocity, acceleration)` in your python file

## Limitations <a name="limitations"></a>

* The covariances are only transformed along the diagonal at the moment
* The python binding only works for python2.7 as this is a boost limitation

## Building and Testing <a name="building-and-testing"></a>

The package can be build with `catkin build` if it was integrated in a catkin workspace.

This repository contains unit-, nose-, and integration tests.
* Unit tests test the cpp functionality
* nose tests test the python binding
* integration tests test the ROS nodelet

All of those tests can be executed with `catkin run_tests`.
The tests are also executed on our perception kit Jenkins pipeline.

## Contribution Guidelines and Feedback <a name="contributing"></a>

Your contribution and feedback is welcome! Simply file a Pull Request on bitbucket. For more information see the
[contribution document](CONTRIBUTING.md), which is basically saying the same than the BIOS guides and rules.

## About <a name="about"></a>

Please contact the maintainers below personally by email or chat. Alternatively, you can post in our
[team room "Multimodal Perception"](https://teams.microsoft.com/l/team/19%3a643256d50e6543f797fc82c46bc2ebca%40thread.skype/conversations?groupId=0947c8fd-85c7-4b51-92d8-025c200d9b22&tenantId=0ae51e19-07c8-4e4b-bb6d-648ee58410f4). Join it using the team code ```xnq6w9u```.

|                    |         |
| -------------------|---------|
| Maintainers        | <a href="mailto:tobias.baer3@de.bosch.com">Tobias Bär</a> |
| Used Encryption    | No custom encryption is used (see BIOS Repository Policy §4.a) |
| 3rd party licenses | None    |

### License <a name="license"></a>

[![License: BIOSL v4](http://bios.intranet.bosch.com/bioslv4-badge.svg)](#license)

> Copyright (c) 2009, 2018 Robert Bosch GmbH and its subsidiaries.
> This program and the accompanying materials are made available under
> the terms of the Bosch Internal Open Source License v4
> which accompanies this distribution, and is available at
> http://bios.intranet.bosch.com/bioslv4.txt

<!---

        Copyright (c) 2009, 2018 Robert Bosch GmbH and its subsidiaries.
        This program and the accompanying materials are made available under
        the terms of the Bosch Internal Open Source License v4
        which accompanies this distribution, and is available at
        http://bios.intranet.bosch.com/bioslv4.txt

-->