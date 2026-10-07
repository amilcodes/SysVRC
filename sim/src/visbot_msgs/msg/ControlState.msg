# What the controller thinks is going on, published at ~30 Hz for the dashboard.
std_msgs/Header header

# odometry estimate (field frame, inches, compass heading)
float64 x
float64 y
float64 theta_deg

# mission progress
string  mission
string[] step_labels         # human-readable definition of every step
int32   step_index
int32   step_count
string  step_name
float64 step_error
float64 step_elapsed_ms
float64 mission_ms           # since the routine started (sim clock); stops when done
string  last_exit
string  mode                 # drive | turn | swing | point_to_point | disabled
bool    done
bool    interfered           # ez::interfered - a motion exited on velocity/timeout
string  last_action          # most recent mechanism call (intake, clamp, doinker...)
float64 last_action_at_ms

# raw sensors seen this tick
float64 imu_heading_deg
float64 enc_left_deg
float64 enc_right_deg
float64 sensor_age_us

# output (V5 units, -127..127) and the Twist actually sent
float64 cmd_left
float64 cmd_right
float64 cmd_linear_mps
float64 cmd_angular_rps
