Navigating with Jackal
======================

Below are the example launch files for three different configurations for navigating Jackal:

- Navigation in an odometric frame without a map, using move_base.
- Generating a map using gmapping.
- Localization with a known map using amcl.

If you're using simulation, bring up Jackal with the front laser enabled for the following demos.

If you're working with a real Jackal, it's suggested to connect via SSH and launch the jackal_navigation
launchfiles from on board the robot. You'll need to have bidirectional communication with the robot's
roscore in order to launch rviz on your workstation, of course (see :ref:`network`).


Navigation Without a Map
------------------------

Making a Map
------------

Navigation With a Map
---------------------
