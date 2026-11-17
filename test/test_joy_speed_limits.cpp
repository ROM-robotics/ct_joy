#include "joy_speed_limits.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void test_yaml_values()
{
  const auto limits = ct_joy::load_joy_speed_limits();
  require(limits.horizontal_speed == 12.0, "horizontal_speed loaded");
  require(limits.d_gain == 1.0, "d_gain loaded");
  require(limits.position_mode_horizontal_speed == 10.0, "position speed loaded");
  require(limits.z_vel_max_speed_up == 3.0, "climb speed loaded");
  require(limits.z_vel_max_speed_down == 1.5, "descent speed loaded");
}

void test_yaml_keys_are_locked()
{
  rclcpp::Node node(
    "joy_speed_limit_test",
    rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("position_mode_horizontal_speed", 20.0)}));
  bool rejected = false;
  try {
    ct_joy::reject_speed_limit_overrides(node);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, "YAML parameter override rejected");
}

void test_non_yaml_parameters_are_allowed()
{
  rclcpp::Node node(
    "joy_runtime_parameter_test",
    rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("max_horizontal_speed_mps", 2.0)}));
  ct_joy::reject_speed_limit_overrides(node);
}

void test_horizontal_velocity_is_magnitude_clamped()
{
  double vx = -8.0;
  double vy = 6.0;
  ct_joy::clamp_horizontal_velocity(vx, vy, 5.0);
  require(std::abs(vx + 4.0) < 1e-9, "Horizontal clamp preserves x direction");
  require(std::abs(vy - 3.0) < 1e-9, "Horizontal clamp preserves y direction");
  require(std::abs(std::hypot(vx, vy) - 5.0) < 1e-9, "XY magnitude is capped");

  vx = 3.0;
  vy = 4.0;
  ct_joy::clamp_horizontal_velocity(vx, vy, 5.0);
  require(vx == 3.0 && vy == 4.0, "Commands below the cap are unchanged");
}

void test_command_limits()
{
  ct_joy::JoySpeedLimits yaml;
  yaml.horizontal_speed = 12.0;
  yaml.position_mode_horizontal_speed = 10.0;
  yaml.z_vel_max_speed_up = 3.0;
  yaml.z_vel_max_speed_down = 1.5;
  yaml.max_yaw_rate_rad_s = 2.5;
  rclcpp::Node node(
    "joy_command_limit_test",
    rclcpp::NodeOptions().parameter_overrides(
    {
      rclcpp::Parameter("max_horizontal_speed_mps", 20.0),
      rclcpp::Parameter("max_position_mode_horizontal_speed_mps", 8.0),
      rclcpp::Parameter("max_vertical_speed_up_mps", 5.0),
      rclcpp::Parameter("max_vertical_speed_down_mps", 1.0),
      rclcpp::Parameter("max_yaw_rate_rps", 3.0)}));
  const auto limits = ct_joy::declare_command_limits(node, yaml);
  require(limits.horizontal_speed == 12.0, "General horizontal cap comes from YAML");
  require(limits.position_mode_horizontal_speed == 8.0, "Lower position cap is retained");
  require(limits.vertical_speed_up == 3.0, "Upward cap comes from YAML");
  require(limits.vertical_speed_down == 1.0, "Lower downward cap is retained");
  require(limits.yaw_rate_rad_s == 2.5, "Yaw cap comes from YAML");
  require(
    node.get_parameter("max_vertical_speed_up_mps").as_double() == 3.0,
    "Parameter server exposes the clamped value");
  require(
    ct_joy::clamp_vertical_rate(3.0 + 0.8, limits) == 3.0,
    "Combined stick and D-pad climb is capped");
  require(
    ct_joy::clamp_vertical_rate(-1.0 - 0.8, limits) == -1.0,
    "Combined stick and D-pad descent is capped");
  require(
    ct_joy::clamp_vertical_rate(-0.4, limits) == -0.4,
    "Vertical rates below the limit are unchanged");

  rclcpp::Node above_node(
    "joy_above_limit_test",
    rclcpp::NodeOptions().parameter_overrides(
    {
      rclcpp::Parameter("max_position_mode_horizontal_speed_mps", 20.0),
      rclcpp::Parameter("max_vertical_speed_down_mps", 2.0)}));
  const auto above_limits = ct_joy::declare_command_limits(above_node, yaml);
  require(
    above_limits.position_mode_horizontal_speed == 10.0,
    "Position cap comes from YAML");
  require(above_limits.vertical_speed_down == 1.5, "Downward cap comes from YAML");
}

void test_invalid_command_limits_are_rejected()
{
  const auto yaml = ct_joy::load_joy_speed_limits();
  for (const auto & parameter : {
      rclcpp::Parameter("max_horizontal_speed_mps", -1.0),
      rclcpp::Parameter(
        "max_position_mode_horizontal_speed_mps",
        std::numeric_limits<double>::quiet_NaN()),
      rclcpp::Parameter("max_vertical_speed_up_mps", -1.0),
      rclcpp::Parameter("max_vertical_speed_down_mps", -1.0),
      rclcpp::Parameter("max_yaw_rate_rps", -1.0),
      rclcpp::Parameter("max_vertical_speed_mps", 2.0)})
  {
    rclcpp::Node node(
      "joy_invalid_limit_test", rclcpp::NodeOptions().parameter_overrides({parameter}));
    bool rejected = false;
    try {
      ct_joy::declare_command_limits(node, yaml);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "Invalid or obsolete command limit rejected: " + parameter.get_name());
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    test_yaml_values();
    test_yaml_keys_are_locked();
    test_non_yaml_parameters_are_allowed();
    test_horizontal_velocity_is_magnitude_clamped();
    test_command_limits();
    test_invalid_command_limits_are_rejected();
  } catch (const std::exception & error) {
    std::cerr << "ct_joy speed-limit test failed: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
