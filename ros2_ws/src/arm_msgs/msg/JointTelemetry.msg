# What the board reports back about the six joints.

std_msgs/Header header

uint32 device_timestamp_us

# Radians. `commanded` is what the servo was told after the stops and the slew
# limit were applied, which is not what it was asked for and not necessarily
# where it is.
float64[6] commanded_position
float64[6] measured_position
float64[6] measured_velocity

# Microseconds. Zero means the channel has no pulse train and the servo is
# relaxed.
uint16[6] pulse_us

# Estimated, not measured. The board has a shunt for the real thing and no
# board exists. See hardware/semg-servo-board/docs/thermal_budget.md.
float32 bus_current_a
float32 board_temperature_c

# Bit field, matching pa::protocol::fault in the firmware.
uint16 faults
