#include "ct_follower_png/px4_multirotor_follower.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include <rom/tools/CoutTag.h>
// #include "ct_follower_png/rom/rom.hpp"

inline auto& rom_cout = rom::tools::cout;
using rom_color = rom::tools::Color;

namespace ct_follower_png
{

Px4MultirotorFollower::Px4MultirotorFollower(
    rclcpp::Node::SharedPtr node, Px4FollowerConfig config)
    : node_(std::move(node)), config_(std::move(config))
{
    if (!node_) {
        throw std::invalid_argument("PX4 follower requires a ROS node");
    }
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    offboard_pub_ = node_->create_publisher<OffboardControlMode>(
        "/px4_2/fmu/in/offboard_control_mode", 10);
    traj_pub_ = node_->create_publisher<TrajectorySetpoint>(
        "/px4_2/fmu/in/trajectory_setpoint", 10);
    cmd_pub_ = node_->create_publisher<VehicleCommand>(
        "/px4_2/fmu/in/vehicle_command", 10);
    data_pub_ = node_->create_publisher<DataMsg>("/vpng_data", 10);

    // ============================================================
    //  Subscribers -- used for guidance
    // ============================================================


    detect_sub_ = node_->create_subscription<RectMsg>(
        "/camera_detect_result", 10,
        [this](const RectMsg::UniquePtr msg) {
            if (detection_handler_) {
                detection_handler_(DetectionObservation{
                    msg->x, msg->y, msg->width, msg->height,
                    msg->header.stamp.sec, msg->header.stamp.nanosec, steady_now()});
            }
        });
    // Own attitude (quaternion → Euler angles)
    odometry_sub_ = node_->create_subscription<VehicleOdometry>(
        "/px4_2/fmu/out/vehicle_odometry", qos,
        [this](const VehicleOdometry::SharedPtr msg) {
            double q0 = msg->q[0], q1 = msg->q[1];
            double q2 = msg->q[2], q3 = msg->q[3];
            state_.roll  = std::atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
            state_.pitch = std::asin (2*(q0*q2 - q3*q1));
            state_.yaw   = std::atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));

            // Cache state_.body_to_ned here (once per odometry sample) instead of rebuilding it
            // separately inside png_calculate() and update_horizontal_approach_shadow().
            state_.body_to_ned = (Eigen::AngleAxisd(state_.yaw,   Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(state_.pitch, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(state_.roll,  Eigen::Vector3d::UnitX())).toRotationMatrix();
        });

    // Own velocity and local altitude
    local_pos_sub_ = node_->create_subscription<VehicleLocalPosition>(
        "/px4_2/fmu/out/vehicle_local_position_v1", qos,
        [this](const VehicleLocalPosition::SharedPtr msg) {
            state_.vx = msg->vx;
            state_.vy = msg->vy;
            state_.vz = msg->vz;

            state_.z = msg->z;
            state_.local_position_valid = msg->z_valid && msg->v_z_valid &&
                std::isfinite(msg->z) && std::isfinite(msg->vz);

            state_.horizontal_velocity_valid = msg->v_xy_valid &&
                std::isfinite(msg->vx) && std::isfinite(msg->vy);

            state_.vertical_velocity_valid = msg->v_z_valid &&
                std::isfinite(msg->vz);

            ekf_xy_valid_ = msg->xy_valid && std::isfinite(msg->x) && std::isfinite(msg->y);
            ekf_z_valid_ = msg->z_valid && std::isfinite(msg->z);
            ekf_global_ref_valid_ = msg->xy_global && msg->z_global &&
                std::isfinite(msg->ref_lat) && std::isfinite(msg->ref_lon) &&
                std::isfinite(msg->ref_alt);
            update_return_home_validity();
        });

    // Home position (checks whether RTL home is valid)
    home_sub_ = node_->create_subscription<HomePosition>(
        "/px4_2/fmu/out/home_position", qos,
        [this](const HomePosition::SharedPtr msg) {
            home_position_valid_ = msg->valid_alt && msg->valid_hpos && msg->valid_lpos &&
                std::isfinite(msg->x) && std::isfinite(msg->y) && std::isfinite(msg->z);
            update_return_home_validity();
        });

    // Flight status (checks Offboard and RTL)
    status_sub_ = node_->create_subscription<VehicleStatus>(
        "/px4_2/fmu/out/vehicle_status_v1", qos,
        [this](const VehicleStatus::SharedPtr msg) {
            state_.armed = (msg->arming_state == VehicleStatus::ARMING_STATE_ARMED);
            state_.external_control_active =
                (msg->nav_state == msg->NAVIGATION_STATE_OFFBOARD);
            state_.return_home_active =
                (msg->nav_state == msg->NAVIGATION_STATE_AUTO_RTL);
        });

    // Hover thrust estimate (checks take-off completion)
    hover_sub_ = node_->create_subscription<HoverThrustEstimate>(
        "/px4_2/fmu/out/hover_thrust_estimate", qos,
        [this](const HoverThrustEstimate::SharedPtr msg) {
            if (msg->hover_thrust_var < 0.0025f) {
                hover_thrust_    = msg->hover_thrust;
                hover_thrust_ok_ = true;
            }
        });

    // ============================================================
    //  Subscribers -- statistics only (guidance algorithm must never use these)
    // ============================================================

    // Own GPS → establishes the statistics coordinate frame
    self_gps_sub_ = node_->create_subscription<SensorGps>(
        "/px4_2/fmu/out/vehicle_gps_position", qos,
        [this](const SensorGps::SharedPtr msg) {
            if (!lc_stats_init_) {
                // Use the first GPS fix as the LocalCartesian origin (statistics only)
                lc_stats_ = GeographicLib::LocalCartesian(
                    msg->latitude_deg,
                    msg->longitude_deg,
                    msg->altitude_msl_m);
                lc_stats_init_ = true;
#ifdef CSV_LOGGING_DEBUG
            #ifndef USE_ROM_STYLE_LOG
                // RCLCPP_INFO(node_->get_logger(),
                //     "\033[33m[stats] Coordinate frame origin initialized: lat=%.7f lon=%.7f\033[0m",
                //     msg->latitude_deg, msg->longitude_deg);
            #else
                rom_cout << rom_color::Default << "[stats] Coordinate frame origin initialized: lat=" << msg->latitude_deg << " lon=" << msg->longitude_deg;
            #endif
#endif
            }
            // Update own position (ENU → NED)
            double e, n, u;
            lc_stats_.Forward(msg->latitude_deg, msg->longitude_deg,
                              msg->altitude_msl_m, e, n, u);
            self_pos_stats_ = {(float)n, (float)e, (float)(-u)};
        });

    // Target GPS (px4_2, statistics only! guidance algorithm must not use this)
    target_gps_sub_ = node_->create_subscription<SensorGps>(
        "/px4_1/fmu/out/vehicle_gps_position", qos,
        [this](const SensorGps::SharedPtr msg) {
            if (!lc_stats_init_) return;
            double e, n, u;
            lc_stats_.Forward(msg->latitude_deg, msg->longitude_deg,
                              msg->altitude_msl_m, e, n, u);
            target_pos_stats_  = {(float)n, (float)e, (float)(-u)};
            target_pos_ok_    = true;
        });


}

Px4MultirotorFollower::~Px4MultirotorFollower()
{
    if (csv_writer_.is_open()) {
        // Build the summary block; the writer thread appends it after
        // draining all pending rows, then flushes and closes the file.
        std::ostringstream summary;
        summary << "\n# ========== Statistics Summary ==========\n";
        summary << "# total_detect_frames,"    << stats_.total_detect_frames << "\n";
        summary << "# lost_frames_total,"  << stats_.lost_frames_total   << "\n";
        summary << "# closest_distance_m," << stats_.min_distance         << "\n";
        if (stats_.hit_recorded) {
            summary << "# hit_time_s," << std::fixed << std::setprecision(4)
                    << stats_.intercept_time << "\n";
        }
        csv_writer_.close(summary.str());
#ifdef CSV_LOGGING_DEBUG
    #ifndef USE_ROM_STYLE_LOG
        // RCLCPP_INFO(node_->get_logger(), "\033[33m[stats] CSV saved to: %s\033[0m", config_.csv_path.c_str());
    #else
        rom_cout << rom_color::Default << "[stats] CSV saved to: " << config_.csv_path;
    #endif
#endif
    }
#ifndef USE_ROM_STYLE_LOG
    std::cout << "VisionPNG node shut down" << std::endl;
#else
    rom_cout << rom_color::Default << "VisionPNG node shut down";
#endif
}

bool Px4MultirotorFollower::prepare()
{
    // Position control သုံးပြီး local NED (0, 0) မှာ သတ်မှတ်အမြင့်သို့ တက်ရန် setpoint ပို့သည်။
    // NED မှာ z အနုတ်တန်ဖိုးသည် အပေါ်ဘက်ဖြစ်သောကြောင့် config_.standby_altitude ကို အနုတ်ဖြင့် သတ်မှတ်ရသည်။
    publish_offboard_position_mode();
    publish_position_setpoint(0.0f, 0.0f, (float)config_.standby_altitude, (float)state_.yaw);

    // Offboard မဝင်မီ takeoff_frames_ အရေအတွက်အထိ heartbeat နှင့် setpoint ကို ကြိုပို့ထားသည်။
    if (takeoff_counter_ < takeoff_frames_) {
        takeoff_counter_++;
        return false;
    }

    // 200 Hz control loop မှာ tick 200 တစ်ကြိမ် (ခန့်မှန်း ၁ စက္ကန့်) command ပြန်ကြိုးစားသည်။
    // Offboard မဖြစ်သေးလျှင် mode ပြောင်းခိုင်းပြီး၊ active ဖြစ်ကြောင်း feedback ရမှ arm ခိုင်းသည်။
    if ((takeoff_counter_ - takeoff_frames_) % 200 == 0) {
        if (!state_.external_control_active) set_offboard_mode();
        if (state_.external_control_active && !state_.armed) arm();
    }
    takeoff_counter_++;

    // အမြင့်ကွာခြားချက် 0.5 m အောက်နှင့် ဒေါင်လိုက်အမြန်နှုန်း 0.3 m/s အောက်ရောက်လျှင် တက်ခြင်းပြီးစီးသည်။
    // Position valid၊ armed နှင့် Offboard active ဖြစ်ရန်လည်း လိုသည်။ hover_thrust_ok_ ကိုတော့ မစစ်ထားပါ။
    float alt_err = std::abs(state_.z - (float)config_.standby_altitude);
    if (state_.local_position_valid && state_.armed && state_.external_control_active && alt_err < 0.5f && std::abs(state_.vz) < 0.3f) 
    {
        #ifdef TAKEOFF_DEBUG
        #ifndef USE_ROM_STYLE_LOG
            // RCLCPP_INFO(node_->get_logger(),
            // "\033[35m→ Take-off complete z=%.2f hover_thrust=%.3f, entering SEARCHING\033[0m",
            // state_.z, hover_thrust_);
        #else
            rom_cout << rom_color::Default << "→ Take-off complete z=" << state_.z << " hover_thrust=" << hover_thrust_ << ", entering SEARCHING";
        #endif
        #endif
        // SEARCHING သို့ပြောင်းပြီး နောက် control tick မှာ hover လုပ်ကာ valid detection အသစ်ကို စောင့်သည်။
        return true;
    } 
    else 
    {
        // Condition မပြည့်သေးလျှင် TAKE_OFF မှာဆက်နေပြီး debug ဖွင့်ထားပါက တစ်စက္ကန့်တစ်ကြိမ် အခြေအနေပြသည်။
        #ifdef TAKEOFF_DEBUG
        #ifndef USE_ROM_STYLE_LOG
            // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            // "\033[35m[INFO throttle=1 Hz][TAKEOFF] z=%.2f/%.2f vz=%.2f pos_valid=%s armed=%s offboard=%s hover_ok=%s\033[0m",
            // state_.z, (float)config_.standby_altitude,
            // state_.vz, state_.local_position_valid ? "YES" : "NO",
            // state_.armed ? "YES" : "NO", state_.external_control_active ? "YES" : "NO",
            // hover_thrust_ok_ ? "YES" : "NO");
        #else
            rom_cout << rom_color::Default << "[TAKEOFF] z=" << state_.z << " / " << config_.standby_altitude << " vz=" << state_.vz << " pos_valid=" << (state_.local_position_valid ? "YES" : "NO") << " armed=" << (state_.armed ? "YES" : "NO") << " offboard=" << (state_.external_control_active ? "YES" : "NO") << " hover_ok=" << (hover_thrust_ok_ ? "YES" : "NO");
        #endif  
        #endif
    }
    return false;
}

void Px4MultirotorFollower::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void Px4MultirotorFollower::set_offboard_mode()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

void Px4MultirotorFollower::return_home()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
}

void Px4MultirotorFollower::update_return_home_validity()
{
    state_.return_home_valid =
        ekf_xy_valid_ && ekf_z_valid_ &&
        state_.horizontal_velocity_valid && state_.vertical_velocity_valid &&
        (home_position_valid_ || ekf_global_ref_valid_);
}

bool Px4MultirotorFollower::can_return_home() const
{
    return state_.return_home_valid;
}

void Px4MultirotorFollower::publish_offboard_position_mode()
{
    OffboardControlMode msg{};
    msg.position  = true;
    msg.velocity  = false;
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    offboard_pub_->publish(msg);
}

void Px4MultirotorFollower::publish_offboard_velocity_mode()
{
    OffboardControlMode msg{};
    msg.position  = false;
    msg.velocity  = true;
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    offboard_pub_->publish(msg);
}

void Px4MultirotorFollower::publish_position_setpoint(float x, float y, float z, float yaw)
{
    TrajectorySetpoint msg{};
    msg.position  = {x, y, z};
    msg.yaw       = yaw;
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    traj_pub_->publish(msg);
}

void Px4MultirotorFollower::publish_vehicle_command(uint16_t command, float p1, float p2)
{
    VehicleCommand msg{};
    msg.param1           = p1;
    msg.param2           = p2;
    msg.command          = command;
    msg.target_system    = 3;
    msg.target_component = 1;
    msg.source_system    = 1;
    msg.source_component = 1;
    msg.from_external    = true;
    msg.timestamp        = node_->get_clock()->now().nanoseconds() / 1000;
    cmd_pub_->publish(msg);
}

StatisticsResult Px4MultirotorFollower::record(const GuidanceTelemetry &telemetry)
{
    static const char *const kCsvHeader =
        "time_s,state,detect_w,self_x,self_y,self_z,target_x,target_y,target_z,dist_m,"
        "los_angle_v,los_angle_z,d_v_angle_v,d_v_angle_z,ex,ey,vx,vy,vz,requested_vx,requested_vy,requested_vz,vz_png_only,vz_pid_correction,requested_yaw_rate";

    switch (csv_writer_.ensure_open(config_.csv_path, kCsvHeader)) {
    case CsvWriter::OpenResult::Failed:
        if (!csv_error_logged_) {
            csv_error_logged_ = true;
        #ifndef USE_ROM_STYLE_LOG
            // RCLCPP_ERROR(node_->get_logger(), "[stats] Could not open CSV file: %s", config_.csv_path.c_str());
        #else
            rom_cout << rom_color::Red << "[stats] Could not open CSV file: " << config_.csv_path;
        #endif
        }
        return {false, CsvStatus::Error};
    case CsvWriter::OpenResult::Opened:
#ifdef CSV_LOGGING_DEBUG
    #ifndef USE_ROM_STYLE_LOG
        // RCLCPP_INFO(node_->get_logger(), "\033[33m[stats] CSV created: %s\033[0m", config_.csv_path.c_str());
    #else
        rom_cout << rom_color::Default << "[stats] CSV created: " << config_.csv_path;
    #endif
#endif
        break;
    case CsvWriter::OpenResult::AlreadyOpen:
        break;
    }

    const double steady_time =
        std::chrono::duration<double>(steady_now().time_since_epoch()).count();

    // Compute the true distance (statistics only, not used by guidance)
    float dist = -1.0f;
    bool just_hit = false; // marks whether a hit occurred on this frame
    if (target_pos_ok_)
    {
        dist = (target_pos_stats_ - self_pos_stats_).norm();

        // Update the closest-approach distance
        if ((double)dist < stats_.min_distance) 
        {
            stats_.min_distance = (double)dist;
        }

        // Hit detection (statistics layer): only count a hit during active INTERCEPT
        if (telemetry.stage == MissionStage::INTERCEPT &&
            dist < config_.hit_radius && !stats_.hit_recorded) 
        {
            stats_.hit_recorded   = true;
            stats_.intercept_time = steady_time;
            just_hit = true; // mark this frame as the hit frame
        }
    }

    const bool first_rth =
        (telemetry.stage == MissionStage::RETURN_HOME && !stats_.rth_recorded);
    if (first_rth) {
        stats_.rth_recorded = true;
    }

    // Throttling: write one row every 2 calls (~10ms @ 200Hz control_tick, since
    // record() runs inside handle_intercept()) to avoid an oversized file.
    csv_counter_++;
    if ((csv_counter_ % 2 != 0) && !just_hit && !first_rth) {
            return {false, CsvStatus::Skipped}; 
        }

    // Format the row on this (real-time) thread, then hand the finished string
    // off to the writer thread — enqueue_row() never blocks on disk I/O.
    std::ostringstream row;
    row << std::fixed << std::setprecision(4)
        << steady_time << ","
        << mission_stage_name(just_hit ? MissionStage::DONE : telemetry.stage) << ","
        << telemetry.bbox_width << ","
        << self_pos_stats_(0) << "," << self_pos_stats_(1) << "," << -self_pos_stats_(2) << ","
        << target_pos_stats_(0) << "," << target_pos_stats_(1) << "," << -target_pos_stats_(2) << ","
        << dist << ","
        << telemetry.los_angle_v << "," << telemetry.los_angle_z << ","
        << telemetry.desired_angle_v << "," << telemetry.desired_angle_z << ","
        << telemetry.ex << "," << telemetry.ey << ","
        << telemetry.aircraft.vx << "," << telemetry.aircraft.vy << "," << telemetry.aircraft.vz << ","
        << telemetry.requested_vx << "," << telemetry.requested_vy << "," << telemetry.requested_vz << ","
        << telemetry.vz_png_only << "," << telemetry.vz_pid_correction << ","
        << telemetry.yaw_rate;
    csv_writer_.enqueue_row(row.str());

    if (just_hit) {
    #ifndef USE_ROM_STYLE_LOG
        #ifdef HIT_LOG_DEBUG
        // RCLCPP_WARN(node_->get_logger(),
        //     "★ [stats] Hit detected! True distance=%.3f m (statistics only)", dist);
        #endif
    #else
        #ifdef HIT_LOG_DEBUG
        rom_cout << rom_color::Magenta << "★ [stats] HIT! True distance=" << dist
                  << " m < hit_radius=" << config_.hit_radius << " m → DONE (statistics only)";
        #endif
    #endif
    }
    if (first_rth) {
        rom_cout << rom_color::Red << "★ [RTH] Entering RETURN_HOME mode (RTL commanded)";
    }

    if (csv_writer_.has_error()) {
        if (!csv_error_logged_) {
            csv_error_logged_ = true;
            // RCLCPP_ERROR(node_->get_logger(), "[stats] Could not write CSV file: %s",
            //     config_.csv_path.c_str());
        }
        return {just_hit, CsvStatus::Error};
    }
    return {just_hit, CsvStatus::Written};
}

void Px4MultirotorFollower::set_detection_handler(DetectionHandler handler)
{
    detection_handler_ = std::move(handler);
}

void Px4MultirotorFollower::apply(const FollowerAction &action)
{
    std::visit([this](const auto &command) {
        using Command = std::decay_t<decltype(command)>;
        if constexpr (std::is_same_v<Command, PositionSetpoint>) {
            if (!std::isfinite(command.x) || !std::isfinite(command.y) ||
                !std::isfinite(command.z) || !std::isfinite(command.yaw)) {
                throw std::invalid_argument("Nonfinite follower position action");
            }
            publish_position_setpoint(command.x, command.y, command.z, command.yaw);
        } else if constexpr (std::is_same_v<Command, VelocityYawRateSetpoint>) {
            if (!std::isfinite(command.vx) || !std::isfinite(command.vy) ||
                !std::isfinite(command.vz) || !std::isfinite(command.yaw_rate)) {
                throw std::invalid_argument("Nonfinite follower velocity action");
            }
            publish_velocity_setpoint(command.vx, command.vy, command.vz, command.yaw_rate);
        } else {
            switch (command) {
            case FlightRequest::Arm: arm(); break;
            case FlightRequest::EnableExternalControl: set_offboard_mode(); break;
            case FlightRequest::PositionHeartbeat: publish_offboard_position_mode(); break;
            case FlightRequest::VelocityHeartbeat: publish_offboard_velocity_mode(); break;
            case FlightRequest::ReturnHome: return_home(); break;
            default: throw std::invalid_argument("Unsupported follower flight request");
            }
        }
    }, action);
}

void Px4MultirotorFollower::publish_velocity_setpoint(
    float vx, float vy, float vz, float yaw_rate)
{
    TrajectorySetpoint msg{};
    msg.position = {std::nanf(""), std::nanf(""), std::nanf("")};
    msg.yaw = std::nanf("");
    msg.yawspeed = yaw_rate;
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.velocity = {vx, vy, vz};
    traj_pub_->publish(msg);
}

void Px4MultirotorFollower::publish_telemetry(const GuidanceTelemetry &telemetry)
{
    DataMsg data{};
    data.d_v_angle_v = static_cast<float>(telemetry.desired_angle_v);
    data.d_v_angle_z = static_cast<float>(telemetry.desired_angle_z);
    data.v_angle_v = static_cast<float>(telemetry.velocity_angle_v);
    data.v_angle_z = static_cast<float>(telemetry.velocity_angle_z);
    data.los_angle_v = static_cast<float>(telemetry.los_angle_v);
    data.los_angle_z = static_cast<float>(telemetry.los_angle_z);
    data.diff_los_angle_v = static_cast<float>(telemetry.diff_los_angle_v);
    data.diff_los_angle_z = static_cast<float>(telemetry.diff_los_angle_z);
    data.ex = static_cast<float>(telemetry.ex);
    data.ey = static_cast<float>(telemetry.ey);
    data.yaw = static_cast<float>(telemetry.aircraft.yaw);
    data.pitch = static_cast<float>(telemetry.aircraft.pitch);
    data.roll = static_cast<float>(telemetry.aircraft.roll);
    data.pixel_u = static_cast<float>(telemetry.pixel_u);
    data.pixel_v = static_cast<float>(telemetry.pixel_v);
    data.vel_total = static_cast<float>(telemetry.speed);
    data_pub_->publish(data);
    ++stats_.total_detect_frames;
    if (telemetry.bbox_width == -1) {
        ++stats_.lost_frames_total;
    }
}

}  // namespace ct_follower_png
