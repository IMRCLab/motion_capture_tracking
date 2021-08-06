[![ROS2](https://github.com/IMRCLab/motion_capture_tracking/actions/workflows/ROS.yml/badge.svg?branch=ros2)](https://github.com/IMRCLab/motion_capture_tracking/actions/workflows/ROS.yml)

# motion_capture_tracking

This repository is a ROS package that provides an abstraction of different motion capture systems (Vicon, OptiTrack, Qualisys) with three different tracking modes: i) Tracking of rigid bodies poses via the official software (e.g., Vicon Tracker) using unique marker arrangements, ii) Tracking of rigid bodies poses with custom frame-to-frame tracking with identical marker arrangements, iii) Tracking of unlabeled marker positions using custom frame-to-frame tracking.

## Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

```
cd ros_ws/src
git clone --recurse-submodules https://github.com/IMRCLab/motion_capture_tracking
cd ../
colcon build
```