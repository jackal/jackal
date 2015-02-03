^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jackal_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.5.0 (2015-01-20)
------------------
* Support disabling the joystick for simulation when only the interactive markers are desired for teleop.
* Contributors: Mike Purvis

0.4.2 (2015-01-14)
------------------
* Shorten timeout for the controller spawner's shutdown.
* Contributors: Mike Purvis

0.4.1 (2015-01-07)
------------------
* Remove fork of diff_drive_controller.
* Contributors: Mike Purvis

0.4.0 (2014-12-12)
------------------
* added joystick argumant.
* Adding imu0_differential setting (=true) to control.yaml
* Add dep for joint state controller.
* Contributors: Mike Purvis, Shokoofeh Pourmehr, Tom Moore

0.3.0 (2014-09-10)
------------------

0.2.1 (2014-09-10)
------------------
* Depend on diff_drive_controller.
* Contributors: Mike Purvis

0.2.0 (2014-09-09)
------------------
* Add fork of diff_drive_controller.
* Fix run_depend elements.
* Fix remap for the interactive markers.
* New jackal_control package.
  This is launchers and configuration common to simulated and real
  Jackal, including controller, localization, and teleop.
* Contributors: Mike Purvis
