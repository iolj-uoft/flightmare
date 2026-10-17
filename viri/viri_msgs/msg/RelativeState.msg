# Relative state of the target w.r.t. the chaser, estimated by the LTVKF.
# All vectors are expressed in the chaser's gravity-aligned (GA) frame
# (header.frame_id = ga_body_frame): same origin and yaw as base_link, no roll/pitch.
Header header

bool initialized                    # false until the first stereo measurement arrives
time last_measurement_stamp         # image stamp of the last fused stereo measurement

geometry_msgs/Vector3 p_rel         # target position minus chaser position [m]
geometry_msgs/Vector3 v_tgt         # target inertial velocity [m/s]
geometry_msgs/Vector3 v_chaser      # chaser inertial velocity [m/s] (odometry input)
geometry_msgs/Vector3 v_rel         # v_tgt - v_chaser [m/s]

float64 chaser_yaw                  # chaser yaw in the world frame [rad]

# Row-major 6x6 covariance of [p_rel, v_tgt]
float64[36] covariance
