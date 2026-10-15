# Forearm orientation, from the IMU on the board.
#
# Roll and pitch only. There is no magnetometer, so yaw is unobservable and is
# not reported rather than being reported badly.

std_msgs/Header header

geometry_msgs/Vector3 linear_acceleration
geometry_msgs/Vector3 angular_velocity

float64 roll
float64 pitch

# False while the filter is settling and after a dropout longer than its time
# constant.
bool trusted

# True when the arm is still enough that the accelerometer is measuring
# gravity and nothing else.
bool stationary
