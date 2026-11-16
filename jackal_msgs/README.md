## Notes

These messages are the low-level interface between
[Jackal](Robots/Jackal)'s ARM MCU and integrated PC. Most users of
Jackal should be able to use standard ROS interfaces (eg.
[geometry_msgs/msg/Twist](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Twist.html),
[diagnostics](https://index.ros.org/p/diagnostics)) to command and
monitor the robot. Possible exceptions are:

- If you wish to disable
  [diff_drive_controller](https://index.ros.org/p/diff_drive_controller)
  and command the motors directly to the MCU.
- To programmatically monitor system state such as voltage, current,
  battery, faults, etc.
