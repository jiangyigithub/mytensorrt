# Copyright (c) 2019 Robert Bosch GmbH and its subsidiaries.
# This program and the accompanying materials are made available under
# the terms of the Bosch Internal Open Source License v4
# which accompanies this distribution, and is available at
# http://bios.intranet.bosch.com/bioslv4.txt
#
# All warranties with regard to the grant of rights according to
# BIOSL v4 Section 2 are specifically excluded; in particular licensor
# do not warrant the usability, usefulness or practicability of their
# Contributions and non-infringement of third party rights.

# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy and a label, indicating the occupation type of the cell.
# The definition of the grid is based on: nav_msgs/OccupancyGrid

std_msgs/Header header

# MetaData for the map
nav_msgs/MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] occupancy

# The occupancy type of the cells
int8 STATIC=1
int8 POTENTIALLY_DYNAMIC=2
int8 DYNAMIC=4
int8 LOW_HEIGHT=8
int8 PEDESTRIAN=16
int8 VEHICLE=32
int8 CYCLE=64
int8[] classification
