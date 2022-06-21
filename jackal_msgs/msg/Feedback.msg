# This message represents high-frequency feedback from the MCU,
# as necessary to support closed-loop control and thermal monitoring.
# Default publish frequency is 50Hz.

std_msgs/Header header

DriveFeedback[2] drivers

# Commanded control mode, use the TYPE_ constants from jackal_msgs/Drive.
int8 commanded_mode

# Actual control mode. This may differ from the commanded in cases where
# the motor enable is off, the motors are in over-current, etc.
int8 actual_mode
