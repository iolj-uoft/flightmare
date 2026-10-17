#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace viri_guidance {

/// Actuator limits applied to every guidance command (paper Sec. III-G.1).
struct AttitudeLimits {
  double max_tilt = 40.0 * M_PI / 180.0;  // roll and pitch clamp [rad]
  double thrust_min = 9.0;                // mass-normalized collective thrust [m/s^2]
  double thrust_max = 14.0;
};

/// Attitude + thrust command. Roll/pitch are relative to the GA frame, i.e. they are the
/// world roll/pitch of the chaser; yaw is handled as a rate.
struct AttitudeCommand {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw_rate = 0.0;
  double thrust = 0.0;
};

/**
 * Map a desired acceleration (incl. gravity compensation, GA frame) to attitude and thrust.
 *
 * The desired body z-axis is aligned with a_total while holding the current heading (yaw 0 in
 * GA). For R = Ry(pitch) Rx(roll):  z_B = [cos(roll) sin(pitch), -sin(roll), cos(roll) cos(pitch)].
 * Collective thrust is the projection of a_total onto the *current* body z-axis R_GB e_z, so
 * thrust does not over-inflate while the vehicle is still banking. When roll or pitch hits
 * max_tilt the projection would over-deliver vertical thrust (net climb), so thrust is instead
 * a_z / z_B,current.z: the requested vertical acceleration is held and horizontal is sacrificed.
 */
AttitudeCommand mapAccelerationToAttitude(const Eigen::Vector3d& a_total_ga,
                                          const Eigen::Matrix3d& R_GB,
                                          const AttitudeLimits& limits);

/// World attitude for a GA-relative roll/pitch command at the given world yaw.
Eigen::Quaterniond attitudeToQuaternion(double roll, double pitch, double yaw);

/**
 * Proportional attitude controller producing body rates (tilt error via the quaternion error,
 * same structure as the RPG position controller):
 *   q_e = q^-1 q_des,  w = 2 sign(q_e.w) [k_rp q_e.x, k_rp q_e.y, k_yaw q_e.z] + w_ff
 */
Eigen::Vector3d attitudeErrorToBodyRates(const Eigen::Quaterniond& q_current,
                                         const Eigen::Quaterniond& q_desired, double k_rp,
                                         double k_yaw, const Eigen::Vector3d& feedforward);

}  // namespace viri_guidance
