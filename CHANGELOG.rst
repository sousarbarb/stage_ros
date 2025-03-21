^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package stage_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.9.0 (2025-03-12)
------------------
* Hokuyo UST-10LX + UTM-30LX sensor models
* willow-full.world (single robot and no obstacles, willow-full.pgm)
* Fix / in frame ID (not compatible with TF2 / rviz)
* Parameter pub_gt_tf to enable publication of ground-truth TF
* Set header.seq on ROS topic messages (except on TF) with an incremental uint32
  counter (possibly useful to check if algorithms are losing messages or not)
* Launch file for teleop twist keyboard
* Parameters for TF / messages frame IDs
* Parameter to use odometry model or gt pose in odom_msg and base w.r.t. odom
* Parameter to enable TF static publication or not (latter only publishes
  base_footprint w.r.t. odom and, if enabled, odom w.r.t. ground-truth)

1.8.0 (2017-04-30)
------------------
* Now uses Stage's native event loop properly and added reassuring startup output.
* Added a GUI section so that the world starts in a good place.
* Fixed issue such that ranger intensity values are no longer clipped to 256
  See: `#31 <https://github.com/ros-simulation/stage_ros/issues/31>`_
* Contributors: Richard Vaughan, Shane Loretz, William Woodall, gerkey

1.7.5 (2015-09-16)
------------------
* Removed all references to FLTK/Fluid and use the upstream CMake config file instead.
* Added ``reset_positions`` service to stage (adds dependency on ``std_srvs``).
* Contributors: Aurélien Ballier, Daniel Claes, Scott K Logan, William Woodall

1.7.4 (2015-03-04)
------------------
* Added missing -ldl flag on newer versions of Ubuntu
* Contributors: William Woodall

1.7.3 (2015-01-26)
------------------
* Split up ``fltk`` dep into ``libfltk-dev`` and ``fluid``, only ``run_depend``'ing on fluid.
* Now supports multiple robots with multiple sensors.
* Fixed a bug on systems that cannot populate FLTK_INCLUDE_DIRS.
* Updated topurg model from "laser" to "ranger".
* Added -u option to use name property of position models as its namespace instead of "robot_0", "robot_1", etc.
* Contributors: Gustavo Velasco Hernández, Gustavo Velasco-Hernández, Pablo Urcola, Wayne Chang, William Woodall

1.7.2 (2013-09-19)
------------------
* Changed default GUI window size to 600x400
* Added velocity to ground truth odometry
* Fixed tf (yaw component) for the base_link->camera transform.
* Fixed ground truth pose coordinate system

1.7.1 (2013-08-30)
------------------
* Fixing warnings
* Small fixes
* Added RGB+3D-sensor interface (Primesense/Kinect/Xtion).
  * Publishes CameraInfo, depth image, RGBA image, tf (takes world-file pantilt paremeter into account)
  * Supports the "old" configuration (laser+odom) as well as camera+odom, laser+camera+odom and odom-only.
  Fixed laser transform height (previously was hardcoded at 0.15, now it takes robot height into account).
* Introduced changes from https://github.com/rtv/Stage/issues/34 with some changes (does not require lasers to be present and works without cameras).

1.7.0 (2013-06-27 18:15:07 -0700)
---------------------------------
- Initial move over from old repository: https://code.ros.org/svn/ros-pkg/stacks/stage
- Catkinized
- Stage itself is released as a third party package now
- Had to disable velocities in the output odometry as Stage no longer implements it internally.
- Updated rostest
- Updated rviz configurations
