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

For most systems, different tracking modes are available: 

1. Tracking of rigid body poses via the official software (e.g., Vicon Tracker) using **unique marker arrangements**.
2. Tracking of rigid body poses with custom frame-to-frame tracking with **identical marker arrangements**.
3. Tracking of **single unlabeled marker** positions using custom frame-to-frame tracking.

The different modes can be combined, e.g., one can use the official software for some rigid bodies and the custom frame-by-frame tracking for others.

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

### Choice of Tracking Mode

#### Unique Marker Arrangements

With a unique marker arrangement for each rigid body, you rely on the motion capture vendor's software.
This is generally preferred. However, if you have lots of rigid bodies, it can be hard to design enough unique configurations – there are not many places to put a marker on small rigid bodies.

If your arrangements are too similar, motion capture software may not fail gracefully. For example, it may rapidly switch back and forth between recognizing two different objects at a single physical location.

To use this mode, simply enable the rigid body / asset in your vendor's software and do *not* include it in the `rigid_bodies` section of `cfg.yaml`.

#### Identical Marker Arrangement

If more than one rigid body has the same marker arrangement, standard motion capture software will refuse to track them. Instead, motion_capture_tracking can use the raw point cloud from the motion capture system and track the rigid bodies frame-by-frame. Here we use Iterative Closest Point (ICP) to greedily match the known marker arrangements to the pointcloud. There are two main consequences of this option:

  - The initial positions of the rigid bodies must be known, to establish a mapping between the name and physical locations.

  - The tracking is done frame-by-frame, so if markers are occluded for a significant amount of time, the algorithm may not be able to re-establish the ID-location mapping once they are visible again.

To use this mode, disable the rigid body in your vendor's software and add an entry in the `rigid_bodies` section of `cfg.yaml`. Note that you'll need to specify the marker configuration (i.e., where the markers are placed in the local coordinate frame of the robot) and a dynamics configuration (first-order dynamics limits for filtering outliers).

#### Single Marker Arrangement

A special case of duplicated marker arrangements is the case where we only use a single marker per robot. As before, motion_capture_tracking will use the raw point cloud from the motion capture system and track the CFs frame-by-frame. In this mode, we use optimal task assignment at every frame, which makes this mode more robust to motion capture outliers compared to the duplicate marker arrangements. The main disadvantage is that the yaw angle cannot be observed without moving in the xy-plane.

#### Hybrid Mode

The three different modes can be mixed. Note that when mixing identical marker arrangement and single marker arrangement, a different backend for tracking is used, which might be computationally slower and is generally less well tested.

### Vendor-specific Instructions

### Optitrack

There are two possible backends. 

* "optitrack" uses the Direct Depacketizers option. This works on all platforms, but often has compatibility issues with untested Motive versions and doesn't support all features.
* "optitrack_closed_source" uses the official SDK (version 4.1.0) (only available on x64 Linux; distributed as a binary library)

Make sure that you have the following settings in Motive:

menu Edit/Settings/Streaming:
* Enable NatNet
* Use "Transmission Type" Multicast
* Enable Unlabeled Markers and Rigid Bodies
* Use "Up Axis": Z-Axis
* Use the default ports (1510 command, 1511 data)

We recommend that you first try "optitrack" and switch to "optitrack_closed_source" if you encounter any issues. 

### NOKOV

Since the SDK is not publically available, adding the SDK and building from source is required.

1. Place the SDK in `motion_capture_tracking/deps/libmotioncapture/deps/nokov_sdk` (such that there is for example the file `motion_capture_tracking/deps/libmotioncapture/deps/nokov_sdk/lib/libSeekerSDKClient.so`)
2. In `motion_capture_tracking/CMakeLists.txt` change `set(LIBMOTIONCAPTURE_ENABLE_NOKOV OFF)` to `set(LIBMOTIONCAPTURE_ENABLE_NOKOV ON)`
3. Rebuild using `colcon build`

## Technical Background

The ROS package is a wrapper around [libmotioncapture](https://github.com/IMRCLab/libmotioncapture) and [librigidbodytracker](https://github.com/IMRCLab/librigidbodytracker).
The former is a C++ library that provides a unified interface over different motion capture SDKs to gather poses of rigid bodies and/or pointclouds of unlabeled markers.
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

Information about unlabeled marker tracking (optimal assignment with a min-cost max-flow formulation for each frame) and hybrid tracking (CBS-based optimization) is available in

```
@article{cai2025tracking,
  title={Optimal Assignment for Multi-Robot Tracking Using Motion Capture Systems},
  author={Nan Cai and
          Wolfgang Hönig},
  journal={1st German Robotics Conference},
  code={https://github.com/IMRCLab/motion_capture_tracking},
  pdf={https://imrclab.github.io/assets/pdf/cai2025tracking.pdf},
  year={2025},
}
```

## Related Work

These are current alternatives if no custom tracking is needed:

- https://github.com/MOCAP4ROS2-Project (VRPN, Vicon, Optitrack, Qualisys, Technaid each in separate packages)
- https://github.com/ros-drivers/mocap_optitrack (only Optitrack; Direct Depacketizers)
- https://github.com/alvinsunyixiao/vrpn_mocap (only VRPN)
- https://github.com/ros-drivers/vrpn_client_ros (only VRPN)
