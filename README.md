[![ROS2](https://github.com/IMRCLab/motion_capture_tracking/actions/workflows/ROS.yml/badge.svg?branch=ros2)](https://github.com/IMRCLab/motion_capture_tracking/actions/workflows/ROS.yml)

# motion_capture_tracking

This repository is a ROS 2 package that can receive data from various motion capture systems:

- VICON
- Qualisys
- OptiTrack
- VRPN
- NOKOV
- FZMotion
- Motion Analysis

For most systems, three different tracking modes are available: 

1. Tracking of rigid body poses via the official software (e.g., Vicon Tracker) using unique marker arrangements.
2. Tracking of rigid body poses with custom frame-to-frame tracking with identical marker arrangements.
3. Tracking of unlabeled marker positions using custom frame-to-frame tracking.

The data is directly published via tf2 and a `/poses` topic that supports different QoS settings.

This package was originally developed for [Crazyswarm](https://imrclab.github.io/crazyswarm2/) to track up to 50 flying robots.

## Building

To build from source, clone the latest version from this repository into your ROS 2 workspace and compile the package using

```
cd ros_ws/src
git clone --recurse-submodules https://github.com/IMRCLab/motion_capture_tracking
cd ../
colcon build
```

## Usage

```
ros2 launch motion_capture_tracking launch.py
```

The various options can be configured in `config/cfg.yaml`.

## Technical Background

The ROS package is a wrapper around [libmotioncapture](https://github.com/IMRCLab/libmotioncapture) and [librigidbodytracker](https://github.com/IMRCLab/librigidbodytracker).
The former is a C++ library that provides a unified interface over different motion capture SDKs to gather pose informations of rigid bodies and/or pointclouds of unlabeled markers.
The latter is a C++ library that takes the following inputs: i) a first-order dynamics model, ii) initial poses of rigid bodies, and iii) at every frame a point cloud. It outputs for every frame the best estimate of the robots' poses.

Some more information on the rigid body pose tracking is available in

```
@inproceedings{crazyswarm,
  author    = {James A. Preiss* and
               Wolfgang  H\"onig* and
               Gaurav S. Sukhatme and
               Nora Ayanian},
  title     = {Crazyswarm: {A} large nano-quadcopter swarm},
  booktitle = {{IEEE} International Conference on Robotics and Automation ({ICRA})},
  pages     = {3299--3304},
  publisher = {{IEEE}},
  year      = {2017},
  url       = {https://doi.org/10.1109/ICRA.2017.7989376},
  doi       = {10.1109/ICRA.2017.7989376},
  note      = {Software available at \url{https://github.com/USC-ACTLab/crazyswarm}},
}
```

The unlabeled marker tracking is using an optimal assignment with a min-cost max-flow formulation for each frame.

## Related Work

These are current vendor-agnostic alternatives if no custom tracking is needed:

- https://github.com/ros-drivers/vrpn_client_ros
- https://github.com/MOCAP4ROS2-Project
