# This message represents lower-frequency status updates
# Default publish frequency is 1Hz.

std_msgs/Header header

# Robot Hardware ID
string hardware_id

# Firmware commit hash
string firmware_commit

# Times since MCU power-on.
builtin_interfaces/Duration mcu_uptime
builtin_interfaces/Duration connection_uptime

# Temperature of MCU's PCB in Celsius.
float32 pcb_temperature
float32 mcu_temperature
