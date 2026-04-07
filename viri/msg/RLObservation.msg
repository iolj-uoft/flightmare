# RL Observation message
# Contains relative state and recent action history

# Relative position (target - chaser) [m]
float64 rel_pos_x
float64 rel_pos_y
float64 rel_pos_z

# Relative velocity (target - chaser) [m/s]
float64 rel_vel_x
float64 rel_vel_y
float64 rel_vel_z

# Action history - 3 steps of (roll, pitch, yaw_rate, thrust)
# action_hist_1 = 3 steps ago, action_hist_3 = most recent step

float64 action_hist_1_roll
float64 action_hist_1_pitch
float64 action_hist_1_yaw_rate
float64 action_hist_1_thrust

float64 action_hist_2_roll
float64 action_hist_2_pitch
float64 action_hist_2_yaw_rate
float64 action_hist_2_thrust

float64 action_hist_3_roll
float64 action_hist_3_pitch
float64 action_hist_3_yaw_rate
float64 action_hist_3_thrust
