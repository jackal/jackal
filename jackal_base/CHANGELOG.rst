^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jackal_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.6 (2016-09-30)
------------------
* Minor linter fixes to jackal_diagnostic_updater.
* Contributors: Tony Baltovski

0.3.5 (2016-02-22)
------------------

0.3.4 (2016-02-10)
------------------

0.3.3 (2015-02-20)
------------------
* Remove duration cast, was using incorrect method to get time out of clock
* Return from function early when getifaddrs fails to avoid double free.
* Contributors: Mike Purvis, Paul Bovbel

0.3.2 (2015-02-19)
------------------
* Add simple connection detect to enable Jackal's wifi status LED.
* Get elapsed time from monotonic time source
* Contributors: Mike Purvis, Paul Bovbel

0.3.1 (2015-02-03)
------------------

0.3.0 (2015-01-20)
------------------

0.2.2 (2015-01-14)
------------------
* Simplify mag computation.
* Don't output stderr from env hook.
* A new approach to fallback configuration.
* Add more missing dependencies to jackal_base.
* Add default compass configuration and install it.
* Remove sixpair, use system one instead.
* Contributors: Mike Purvis

0.2.1 (2015-01-12)
------------------
* Resolve catkin_lint.
* Contributors: Mike Purvis

0.2.0 (2015-01-12)
------------------
* read mag msg in radian.
* added magnetometer calibration computation scripts.
* Contributors: Shokoofeh Pourmehr

0.1.0 (2014-11-11)
------------------
* Initial release of basic functionality.
* Contributors: Mike Purvis
