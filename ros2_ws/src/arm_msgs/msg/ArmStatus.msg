# The safety layer's view, plus what the control loop is actually costing.

std_msgs/Header header

uint8 STATE_INIT = 0
uint8 STATE_IDLE = 1
uint8 STATE_ACTIVE = 2
uint8 STATE_HOLDING = 3
uint8 STATE_FAULT = 4

uint8 state
string state_name
uint16 faults
string[] fault_names

# True while frames are arriving from the board inside the timeout.
bool link_up

# Measured on the host, over the last reporting interval. The board reports its
# own separately in JointTelemetry; the two are different loops and both matter.
float64 control_period_mean_ms
float64 control_period_max_ms
float64 control_jitter_rms_ms
uint32 control_iterations
uint32 control_overruns
