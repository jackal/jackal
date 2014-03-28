# This message represents high-frequency feedback from the MCU,
# as necessary to support closed-loop control and thermal monitoring.
# Default publish frequency is 50Hz.

Header header

DriveFeedback[2] drivers

# Temperatures 
float32 pcb_temperature
float32 mcu_temperature
