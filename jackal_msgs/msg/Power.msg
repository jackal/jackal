# Robot Power readings

std_msgs/Header header

# AC Power
int8 NOT_APPLICABLE=-1

int8 shore_power_connected  # Indicates if AC power is connected.
int8 battery_connected      # Indicates if battery is connected.
int8 power_12v_user_nominal # Indicates if the 12V user power is good.
int8 charging_complete      # Indicates if charging is complete.

# Voltage rails, in volts
# Averaged over the message period

# Jackal
uint8 JACKAL_MEASURED_BATTERY=0
uint8 JACKAL_MEASURED_5V=1
uint8 JACKAL_MEASURED_12V=2

float32[] measured_voltages

# Current senses available on platform, in amps.
# Averaged over the message period

# Jackal
uint8 JACKAL_TOTAL_CURRENT=0
uint8 JACKAL_COMPUTER_CURRENT=1
uint8 JACKAL_DRIVE_CURRENT=2
uint8 JACKAL_USER_CURRENT=3

float32[] measured_currents
