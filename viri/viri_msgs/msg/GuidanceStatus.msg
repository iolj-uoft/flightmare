# Diagnostic output of the guidance node.
Header header

uint8 IDLE=0
uint8 PRONAV=1
uint8 HEADING=2
uint8 STALE=3
uint8 INTERCEPTED=4
uint8 mode

string law                          # "pronav" or "hybrid"

float64 range                       # |r| [m]
float64 closing_speed               # Vc = -r_hat . v_rel [m/s]
float64 azimuth_error               # eps_psi in the GA frame [rad]
geometry_msgs/Vector3 los_rate      # Omega = r x v_rel / |r|^2 [rad/s]
geometry_msgs/Vector3 accel_cmd     # total commanded acceleration incl. gravity, GA frame [m/s^2]

float64 roll_cmd                    # [rad]
float64 pitch_cmd                   # [rad]
float64 yaw_rate_cmd                # [rad/s]
float64 thrust_cmd                  # mass-normalized collective thrust [m/s^2]
