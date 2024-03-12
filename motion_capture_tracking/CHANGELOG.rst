^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package motion_capture_tracking
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.5 (2024-03-12)
------------------
* Install libNatNet.so only on x64 Linux
  * Should fix ROS 2 build farm errors.
* Contributors: Wolfgang Hoenig

1.0.4 (2024-03-06)
------------------
* install libNatNet.so as part of package
  Fixes `#14 <https://github.com/IMRCLab/motion_capture_tracking/issues/14>`_
* Add support for the "mock" motion capture type
  In "mock" mode, the rigid bodies defined in the cfg.yaml will be published at a fixed rate. This is useful for testing without access to a motion capture system.
* cfg: remove unused "mode"
  Mode is not being used in the code, and is therefore removed from the config.
* Contributors: Wolfgang Hoenig

1.0.3 (2024-01-29)
------------------

1.0.2 (2024-01-23)
------------------

1.0.1 (2024-01-20)
------------------
* initial release
* Contributors: Wolfgang Hönig
