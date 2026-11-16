#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

using namespace std::chrono_literals;

class F710Px4Controller final : public rclcpp::Node
{
public:
  F710Px4Controller()
  : Node("f710_px4_controller"), last_tick_(std::chrono::steady_clock::now())
  {
    joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    joy_timeout_sec_ = declare_parameter<double>("joy_timeout_sec", 0.5);
    takeoff_altitude_m_ = declare_parameter<double>("takeoff_altitude_m", 2.0);
    max_horizontal_speed_mps_ = declare_parameter<double>("max_horizontal_speed_mps", 3.0);
    max_vertical_speed_mps_ = declare_parameter<double>("max_vertical_speed_mps", 1.0);
    max_yaw_rate_rps_ = declare_parameter<double>("max_yaw_rate_rps", 1.5);
    altitude_trim_speed_mps_ = declare_parameter<double>("altitude_trim_speed_mps", 0.8);
    deadzone_ = declare_parameter<double>("deadzone", 0.08);

    invert_vx_axis_ = declare_parameter<bool>("invert_vx_axis", true);
    invert_vy_axis_ = declare_parameter<bool>("invert_vy_axis", false);
    invert_vz_axis_ = declare_parameter<bool>("invert_vz_axis", true);
    invert_yaw_axis_ = declare_parameter<bool>("invert_yaw_axis", true);
    axis_vy_ = declare_parameter<int>("axis_vy", 3);
    axis_vx_ = declare_parameter<int>("axis_vx", 4);
    axis_yaw_ = declare_parameter<int>("axis_yaw", 0);
    axis_vz_ = declare_parameter<int>("axis_vz", 1);
    axis_dpad_vertical_ = declare_parameter<int>("axis_dpad_vertical", 7);
    button_arm_ = declare_parameter<int>("button_arm", 2);
    button_disarm_ = declare_parameter<int>("button_disarm", 1);
    button_takeoff_ = declare_parameter<int>("button_takeoff", 3);
    button_land_ = declare_parameter<int>("button_land", 0);

    if (publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be greater than zero");
    }

    const auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort()
      .transient_local();
    offboard_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "fmu/in/offboard_control_mode", px4_qos);
    setpoint_publisher_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "fmu/in/trajectory_setpoint", px4_qos);
    command_publisher_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "fmu/in/vehicle_command", px4_qos);

    joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::QoS(10),
      std::bind(&F710Px4Controller::on_joy, this, std::placeholders::_1));
    local_position_subscription_ =
      create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "fmu/out/vehicle_local_position", px4_qos,
      std::bind(&F710Px4Controller::on_local_position, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&F710Px4Controller::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "Listening to %s; X: arm/Offboard, B: disarm, Y: takeoff, A: land.",
      joy_topic_.c_str());
  }

private:
  static double axis_value(const std::vector<float> & axes, int index, bool invert)
  {
    if (index < 0 || static_cast<std::size_t>(index) >= axes.size()) {
      return 0.0;
    }
    const double value = axes[static_cast<std::size_t>(index)];
    return invert ? -value : value;
  }

  double shape(double value) const
  {
    value = std::clamp(value, -1.0, 1.0);
    if (std::abs(value) <= deadzone_) {
      return 0.0;
    }
    return std::copysign((std::abs(value) - deadzone_) / (1.0 - deadzone_), value);
  }

  static bool button_pressed(const std::vector<int32_t> & buttons, int index)
  {
    return index >= 0 && static_cast<std::size_t>(index) < buttons.size() &&
           buttons[static_cast<std::size_t>(index)] != 0;
  }

  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_time_ = std::chrono::steady_clock::now();
    vx_ = shape(axis_value(msg->axes, axis_vx_, invert_vx_axis_)) *
      max_horizontal_speed_mps_;
    vy_ = shape(axis_value(msg->axes, axis_vy_, invert_vy_axis_)) *
      max_horizontal_speed_mps_;
    vz_ = shape(axis_value(msg->axes, axis_vz_, invert_vz_axis_)) *
      max_vertical_speed_mps_;
    yaw_rate_ = shape(axis_value(msg->axes, axis_yaw_, invert_yaw_axis_)) *
      max_yaw_rate_rps_;
    dpad_altitude_rate_ = shape(axis_value(msg->axes, axis_dpad_vertical_, false)) *
      altitude_trim_speed_mps_;

    const bool arm_pressed = button_pressed(msg->buttons, button_arm_);
    const bool disarm_pressed = button_pressed(msg->buttons, button_disarm_);
    const bool takeoff_pressed = button_pressed(msg->buttons, button_takeoff_);
    const bool land_pressed = button_pressed(msg->buttons, button_land_);
    const bool arm_edge = arm_pressed && !button_pressed(last_buttons_, button_arm_);
    const bool disarm_edge = disarm_pressed && !button_pressed(last_buttons_, button_disarm_);
    const bool takeoff_edge = takeoff_pressed && !button_pressed(last_buttons_, button_takeoff_);
    const bool land_edge = land_pressed && !button_pressed(last_buttons_, button_land_);

    if (arm_edge) {
      arm_requested_ = true;
      arm_sequence_ticks_ = 0;
      RCLCPP_INFO(get_logger(), "Arm requested; streaming setpoints before mode/arm commands.");
    }
    if (disarm_edge) {
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0F);
      RCLCPP_INFO(get_logger(), "Disarm requested.");
    }
    if (takeoff_edge) {
      takeoff_requested_ = true;
      if (!arm_requested_) {
        arm_requested_ = true;
        arm_sequence_ticks_ = 0;
      }
      RCLCPP_INFO(get_logger(), "Takeoff requested to %.1f m.", takeoff_altitude_m_);
    }
    if (land_edge) {
      publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
      RCLCPP_INFO(get_logger(), "Land requested.");
    }
    last_buttons_ = msg->buttons;
  }

  void on_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    if (!have_local_position_) {
      altitude_m_ = std::max(0.0, -static_cast<double>(msg->z));
      yaw_rad_ = msg->heading;
      have_local_position_ = true;
    }
  }

  uint64_t timestamp_us()
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  void publish_vehicle_command(uint16_t command, float param1 = 0.0F, float param2 = 0.0F)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = timestamp_us();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    command_publisher_->publish(msg);
  }

  void on_timer()
  {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::clamp(
      std::chrono::duration<double>(now - last_tick_).count(), 0.0, 0.1);
    last_tick_ = now;

    const bool joy_timed_out = !last_joy_time_.has_value() ||
      std::chrono::duration<double>(now - *last_joy_time_).count() > joy_timeout_sec_;
    if (joy_timed_out) {
      vx_ = vy_ = vz_ = yaw_rate_ = dpad_altitude_rate_ = 0.0;
    }

    altitude_m_ = std::max(0.0, altitude_m_ + vz_ * dt);
    if (!joy_timed_out) {
      altitude_m_ = std::max(0.0, altitude_m_ + dpad_altitude_rate_ * dt);
    }
    if (takeoff_requested_) {
      altitude_m_ = takeoff_altitude_m_;
      takeoff_requested_ = false;
    }

    yaw_rad_ = std::atan2(
      std::sin(yaw_rad_ + yaw_rate_ * dt), std::cos(yaw_rad_ + yaw_rate_ * dt));

    px4_msgs::msg::OffboardControlMode mode{};
    mode.timestamp = timestamp_us();
    mode.position = true;
    offboard_publisher_->publish(mode);

    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.timestamp = mode.timestamp;
    const auto nan = std::numeric_limits<float>::quiet_NaN();
    setpoint.position = {nan, nan, static_cast<float>(-altitude_m_)};
    setpoint.velocity = {static_cast<float>(vx_), static_cast<float>(vy_), nan};
    setpoint.yaw = static_cast<float>(yaw_rad_);
    setpoint.yawspeed = nan;
    setpoint_publisher_->publish(setpoint);

    if (arm_requested_ && have_local_position_) {
      ++arm_sequence_ticks_;
      const auto required_ticks = std::max(
        10, static_cast<int>(std::ceil(publish_rate_hz_)));
      if (arm_sequence_ticks_ == required_ticks) {
        // PX4 custom main mode OFFBOARD is 6.
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
        publish_vehicle_command(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
        arm_requested_ = false;
      }
    }
  }

  std::string joy_topic_;
  double publish_rate_hz_{20.0};
  double joy_timeout_sec_{0.5};
  double takeoff_altitude_m_{2.0};
  double max_horizontal_speed_mps_{3.0};
  double max_vertical_speed_mps_{1.0};
  double max_yaw_rate_rps_{1.5};
  double altitude_trim_speed_mps_{0.8};
  double deadzone_{0.08};
  bool invert_vx_axis_{true};
  bool invert_vy_axis_{false};
  bool invert_vz_axis_{true};
  bool invert_yaw_axis_{true};
  int axis_vy_{3};
  int axis_vx_{4};
  int axis_yaw_{0};
  int axis_vz_{1};
  int axis_dpad_vertical_{7};
  int button_arm_{2};
  int button_disarm_{1};
  int button_takeoff_{3};
  int button_land_{0};

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    local_position_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::chrono::steady_clock::time_point last_tick_;
  std::optional<std::chrono::steady_clock::time_point> last_joy_time_;
  std::vector<int32_t> last_buttons_;
  double altitude_m_{0.0};
  double yaw_rad_{0.0};
  double vx_{0.0};
  double vy_{0.0};
  double vz_{0.0};
  double yaw_rate_{0.0};
  double dpad_altitude_rate_{0.0};
  bool have_local_position_{false};
  bool arm_requested_{false};
  int arm_sequence_ticks_{0};
  bool takeoff_requested_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<F710Px4Controller>());
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(rclcpp::get_logger("f710_px4_controller"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
