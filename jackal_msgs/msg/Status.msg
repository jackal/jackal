# This message represents Jackal's lower-frequency status updates
# Default publish frequency is 1Hz.

Header header

# Monitoring the stop loop
bool drivers_active
bool driver_external_stop_present
bool driver_external_stop_stopped

# Voltage rails, in volts
# Averaged over the message period
float32 measured_battery
float32 measured_12v
float32 measured_5v

# Current senses available on platform, in amps.
# Averaged over the message period
float32 drive_current
float32 user_current
float32 computer_current
float32 total_current

# Highest total system current peak as measured in a 1ms window.
float32 total_current_peak

# Integration of all power consumption since MCU power-on, in watt-hours.
float64 total_power_consumed 
