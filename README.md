# Simple Pointcloud Mapper (ROS2)

## Overview

The **Simple PointCloud Mapper** is a ROS2 node that subscribes to a point cloud topic and an odometry topic to build a 3D map of the environment. The generated map is published as a point cloud.

## Assumptions

The input point cloud is registered and in a fixed frame (`camera_init`).

## Features

- Subscribes to a point cloud topic (/cloud_registered by default).
- Subscribes to an odometry topic (/Odometry by default).
- Filters and accumulates point clouds to construct a map.
- Applies voxel grid filtering to downsample the map.
- Saves the final map to a PCD file upon request.

## Installation

### Prerequisites

Ensure you have ROS2 Humble (or later) and PCL installed:

```
sudo apt update && sudo apt install ros-humble-pcl-ros
```

### Build Instructions

```
cd ~/ros2_ws/src
git clone git@github.com:DavDori/Simple-Pointcloud-Mapper-ros2.git simple_pointcloud_mapper
cd ~/ros2_ws
colcon build --packages-select simple_pointcloud_mapper
source install/setup.bash
```

## Usage

To start the mapper node:

```
ros2 run simple_pointcloud_mapper mapper_node
```

## Parameters

The following parameters can be set via the ROS2 parameter server:

```yaml
parameters:
  topic.pointcloud.in: "/cloud_registered"  # Input point cloud topic
  topic.odometry.in: "/Odometry"            # Input odometry topic
  voxel_size_m: 0.1                         # Voxel filter size in meters
  max_range_m: 5.0                          # Maximum range for points in meters
  min_range_m: 1.0                          # Minimum range for points in meters
  filter_step: 50                           # Apply voxel filter every N frames
  path: "./"                                # Save path for the map
  name: "close.pcd"                         # Output PCD file name
```

## Saving the Map

To save the generated point cloud map, call the provided ROS2 service:

```
ros2 service call /save_simple_map std_srvs/srv/Trigger {}
```

This will save the map to the specified `path` with the filename `name`.
