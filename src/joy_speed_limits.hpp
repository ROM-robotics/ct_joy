#pragma once

#include <rclcpp/node.hpp>

namespace ct_joy
{

// Vehicle speed caps loaded from config/params.yaml, mirroring PX4's
// MPC_XY_VEL_MAX / MPC_Z_VEL_MAX_UP / MPC_Z_VEL_MAX_DN / MPC_MAN_Y_MAX
// parameters. `max_yaw_rate_rad_s` is converted from the YAML's
// mpc_man_y_max (degrees/s, Manual mode -- what a human joystick pilot
// commands) to radians/s.
struct JoySpeedLimits
{
	double horizontal_speed = 0.0;
	double z_vel_max_speed_up = 0.0;
	double z_vel_max_speed_down = 0.0;
	double max_yaw_rate_rad_s = 0.0;
};

// Loads config/params.yaml directly from the ct_joy package share directory
// (not via ROS parameter overrides/launch files). Throws
// std::invalid_argument if the file is missing, malformed, or any field is
// missing/non-finite/non-positive. mpc_yawrate_auto_max (Auto/Offboard mode
// yaw rate) is validated but not used here; it is the autonomous-search
// counterpart read by ct_searching_algorithms.
JoySpeedLimits load_joy_speed_limits();

// Throws std::invalid_argument if `node` was given a ROS parameter override
// for horizontal_speed, z_vel_max_speed_up, z_vel_max_speed_down,
// mpc_yawrate_auto_max, or mpc_man_y_max; those may only be set by editing
// config/params.yaml, never via `--ros-args -p` or a launch file.
void reject_speed_limit_overrides(rclcpp::Node & node);

// Clamps a requested vertical rate (m/s, NED-style sign: positive = climb)
// asymmetrically: positive values are capped at limits.z_vel_max_speed_up,
// negative values at -limits.z_vel_max_speed_down.
double clamp_vertical_rate(double rate_mps, const JoySpeedLimits & limits);

}  // namespace ct_joy
