# This message represents a low-level motor command to Jackal.

# Command units dependent on the value of this field
int8 TYPE_SPEED=0   # speed command (rad/s)
int8 TYPE_PWM=1     # proportion of full voltage command [-1.0..1.0]
int8 TYPE_TORQUE=2  # torque command (Nm)
int8 type

# Units as above, +ve direction propels chassis forward.
int8 LEFT=0
int8 RIGHT=1
float32[2] drivers
