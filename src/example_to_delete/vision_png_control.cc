#include "ct_follower_png/vision_png_control.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <rom/tools/CoutTag.h>
// #include "ct_follower_png/rom/rom.hpp"

inline auto& rom_cout = rom::tools::cout;
using rom_color = rom::tools::Color;

namespace ct_follower_png
{

namespace
{
constexpr double search_yaw_rate_command = 0.2;
constexpr unsigned int hover_delay_seconds = 3;
}

VisionPNGGuidance::VisionPNGGuidance(
    std::unique_ptr<Follower> follower, VisionPNGConfig config, Logger &log)
    : follower_(std::move(follower)), logger_(log), config_(std::move(config))
{
    if (!follower_) 
    {
        throw std::invalid_argument("VisionPNGGuidance requires a Follower");
    }

    const auto capabilities = follower_->capabilities();
    if (!capabilities.position_hold || !capabilities.velocity_yaw_rate || !capabilities.hover) 
    {
        throw std::invalid_argument("VisionPNGGuidance requires position hold, velocity/yaw-rate and hover support");
    }
    if (!std::isfinite(config_.coast_thresh_timeout_seconds) ||
        config_.coast_thresh_timeout_seconds < 0.0 ||
        config_.coast_thresh_timeout_seconds > config_.detection_timeout_seconds ||
        !std::isfinite(config_.lost_thresh_timeout_seconds) ||
        config_.lost_thresh_timeout_seconds <= config_.detection_timeout_seconds)
    {
        throw std::invalid_argument("Invalid detection loss durations");
    }
    if (!std::isfinite(config_.re_search_yaw_rate_after_track_lost_rd_per_sec) ||
        config_.re_search_yaw_rate_after_track_lost_rd_per_sec <= 0.0 ||
        config_.re_search_yaw_rate_after_track_lost_rd_per_sec > 3.0)
    {
        throw std::invalid_argument("Invalid track-lost re-search yaw rate");
    }
    if (!std::isfinite(config_.return_home_timeout_seconds) ||
        config_.return_home_timeout_seconds <= 0.0)
    {
        throw std::invalid_argument("Invalid return-home timeout");
    }
    cx_ = config_.image_width / 2.0;
    cy_ = config_.image_height / 2.0;

    yaw_pid_.set_gains({config_.control_wz_gain_1_p, config_.control_wz_gain_2_i, config_.control_wz_gain_3_d});
    vz_pid_.set_gains({config_.control_vz_gain_1_p, config_.control_vz_gain_2_i, config_.control_vz_gain_3_d});

    if (!std::isfinite(config_.approach_bbox_width) || config_.approach_bbox_width <= 0.0 ||
        config_.approach_bbox_width >= config_.image_width ||
        !std::isfinite(config_.approach_resume_bbox_width) || config_.approach_resume_bbox_width <= 0.0 ||
        config_.approach_resume_bbox_width >= config_.approach_bbox_width ||
        !std::isfinite(config_.approach_resume_confirm_seconds) || config_.approach_resume_confirm_seconds <= 0.0 ||
        !std::isfinite(config_.approach_kp) || config_.approach_kp < 0.0 ||
        !std::isfinite(config_.approach_max_speed) || config_.approach_max_speed < 0.0 ||
        !std::isfinite(config_.approach_accel_limit) || config_.approach_accel_limit <= 0.0 ||
        !std::isfinite(config_.hybrid_blend_start_width) || config_.hybrid_blend_start_width < 0.0 ||
        config_.hybrid_blend_start_width >= config_.approach_bbox_width) 
    {
        throw std::invalid_argument("Invalid horizontal approach parameters");
    }


    R_c2b_ << 0, 0, 1,
               1, 0, 0,
               0, 1, 0;

    follower_->set_detection_handler([this](const DetectionObservation &observation) { on_detection(observation); });
}

VisionPNGGuidance::~VisionPNGGuidance()
{
    follower_->set_detection_handler({});
}

void VisionPNGGuidance::on_detection(const DetectionObservation &observation)
{
    if (state_ == MissionStage::RETURN_HOME || state_ == MissionStage::DONE)
    {
        return;
    }

    const auto received_at = observation.received_at;

    if (observation.width != -1)
    {
        const auto& stamp = observation;

        if (stamp.stamp_sec < 0 || stamp.stamp_nanosec >= 1000000000u ||
            (stamp.stamp_sec == 0 && stamp.stamp_nanosec == 0))
        {
            logger_.log_throttled(LogLevel::Warning, {"guidance", "invalid_timestamp"}, 1000,
                "Ignoring detected bbox: invalid measurement timestamp");
            return;
        }

        const auto sample_duration = std::chrono::seconds(stamp.stamp_sec) + std::chrono::nanoseconds(stamp.stamp_nanosec);

        const auto sample_at = std::chrono::steady_clock::time_point(
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                sample_duration));

        if (sample_at > received_at)
        {
            logger_.log_throttled(LogLevel::Warning, {"guidance", "future_timestamp"}, 1000,
                "Ignoring detected bbox: future timestamp; check clock domain");
            return;
        }

        if (has_detection_sample_ && sample_at < detection_sample_at_)
        {
            logger_.log_throttled(LogLevel::Warning, {"guidance", "out_of_order_timestamp"}, 1000,
                "Ignoring detected bbox: out-of-order measurement timestamp");
            return;
        }

        if (received_at - sample_at > detection_timeout_)
        {
            logger_.log_throttled(LogLevel::Warning, {"guidance", "stale_timestamp"}, 1000,
                "Ignoring detected bbox: measurement older than %.6f seconds",
                config_.detection_timeout_seconds);
            return;
        }

        detection_sample_at_ = sample_at;
        has_detection_sample_ = true;
    }

    detect_x_ = observation.x;
    detect_y_ = observation.y;
    detect_w_ = observation.width;
    detect_h_ = observation.height;

    if (observation.width == -1)
    {
        detect_fresh_ = false;
        yaw_detect_fresh_ = false;
    }
    else
    {
        detect_fresh_ =
            !vz_pid_has_consumed_sample_ ||
            detection_sample_at_ > vz_pid_last_consumed_sample_at_;
        yaw_detect_fresh_ =
            !yaw_pid_has_consumed_sample_ ||
            detection_sample_at_ > yaw_pid_last_consumed_sample_at_;
    }

    detection_received_at_ = received_at;

    if (observation.width == -1)                           // Target not detected
    {
        vz_pid_.reset();
        rom_vz_ = 0.0;
        reset_yaw_pid();
#if USE_BC_V2_DEBIASED_MODEL
        reset_bc_v2_state();
#endif
        lost_frames_++;
        #ifdef DETECTION_DEBUG
            #ifndef USE_ROM_STYLE_LOG
                logger_.logf(LogLevel::Info, rom_color::Yellow, "→ Not detected from Tracker");
            #else
                rom_cout << rom_color::Yellow << "→ Not detected from Tracker";
            #endif
        #endif
    } 
    else                                           // Target detected      
    {
        #ifdef DETECTION_DEBUG
            #ifdef USE_ROM_STYLE_LOG
                rom_cout << rom_color::DarkGreen << "→ Target visually detected";
            #endif
        #endif
        const double detected_ex = (detect_x_ + detect_w_ / 2.0) - cx_;
        if (detected_ex < 0.0) {
            track_lost_search_sign_ = -1.0;
        } else if (detected_ex > 0.0) {
            track_lost_search_sign_ = 1.0;
        }
        lost_frames_ = 0;
        // Valid detection: if currently SEARCHING, switch to intercept
        if (state_ == MissionStage::SEARCHING) 
        {
            reset_yaw_pid();
            #ifdef DETECTION_DEBUG
                #ifdef FIRST_INTERCEPT_DEBUG
                    #ifndef USE_ROM_STYLE_LOG
                        logger_.logf(LogLevel::Info, rom_color::DarkGreen, "→ Target visually detected, entering INTERCEPT");
                    #else
                        rom_cout << rom_color::DarkGreen << "→ Target visually detected, entering INTERCEPT";
                    #endif
                #endif
            #endif // DETECTION_DEBUG
            state_ = MissionStage::INTERCEPT;
        }
        // Target reappeared (recovering from TRACK_LOST)
        if (state_ == MissionStage::TRACK_LOST) 
        {
            reset_yaw_pid();
            #ifdef DETECTION_DEBUG
                #ifdef REINTERCEPT_DEBUG
                    #ifndef USE_ROM_STYLE_LOG
                        logger_.logf(LogLevel::Info, rom_color::BrightGreen, "→ Target reappeared, resuming INTERCEPT");
                    #else
                        rom_cout << rom_color::BrightGreen << "→ Target reappeared, resuming INTERCEPT";
                    #endif
                #endif // REINTERCEPT_DEBUG
            #endif // DETECTION_DEBUG
            png_initialized_ = false;  // Re-initialize PNG
#if USE_BC_V2_DEBIASED_MODEL
            reset_bc_v2_state();
#endif
            return_home_commanded_ = false;
            return_home_counter_ = 0;
            state_ = MissionStage::INTERCEPT;
        }
    }
}

void VisionPNGGuidance::control_tick()
{
    aircraft_ = follower_->state();
    last_vz_png_only_ = 0.0f;
    last_vz_pid_correction_ = 0.0f;
    switch (state_) {
    case MissionStage::TAKE_OFF:    handle_takeoff();    break;
    case MissionStage::SEARCHING:   handle_searching();  break;
    case MissionStage::INTERCEPT:   handle_intercept();  break;
    case MissionStage::TRACK_LOST:  handle_track_lost(); break;
    case MissionStage::DONE:        handle_done();       break;
    case MissionStage::RETURN_HOME: handle_return_home(); break;
    }
}

bool VisionPNGGuidance::reset_if_detection_timed_out()
{
    const auto now = follower_->steady_now();
    if (has_detection_sample_ &&
        detection_sample_at_ <= now &&
        now - detection_sample_at_ <= detection_timeout_)
    {
        return false;
    }

    logger_.log_throttled(LogLevel::Warning, {"guidance", "vision_timeout"}, 1000,
        "Vision measurement unavailable or expired: commanding zero velocity and yaw rate");

    reset_yaw_pid();
    yaw_detect_fresh_ = false;
    d_yaw_ = 0.0;

    vz_pid_.reset();
    vz_pid_previous_sample_at_ = {};
    detect_fresh_ = false;
    rom_vz_ = 0.0;

    png_initialized_ = false;
    d_v_angle_v_ = 0.0;
    d_v_angle_z_ = 0.0;
#if USE_BC_V2_DEBIASED_MODEL
    reset_bc_v2_state();
#endif

    coast_vx_cmd_ = 0.0f;
    coast_vy_cmd_ = 0.0f;
    coast_vz_cmd_ = 0.0f;
    coast_yaw_ = 0.0f;
    last_published_velocity_command_ = {};
    last_vz_png_only_ = 0.0f;
    last_vz_pid_correction_ = 0.0f;
    has_published_velocity_command_ = false;
    invalid_velocity_ticks_ = 0;

    approach_width_initialized_ = false;
    approach_stop_latched_ = true;
    approach_resume_pending_ = false;
    approach_speed_ = 0.0;
    approach_vx_ = 0.0f;
    approach_vy_ = 0.0f;

    return true;
}

void VisionPNGGuidance::update_horizontal_approach_shadow()
{
    const auto now = follower_->steady_now();
    const double elapsed = approach_update_at_ == std::chrono::steady_clock::time_point{}
        ? 0.0 : std::chrono::duration<double>(now - approach_update_at_).count();
    approach_update_at_ = now;
    const double dt = std::clamp(elapsed, 0.0, 0.05);
    const double sample_age = std::chrono::duration<double>(now - detection_received_at_).count();
    // cx_/cy_ cached once in the constructor (see vision_png_control.hpp).
    const double pixel_error_x = detect_x_ + detect_w_ / 2.0 - cx_;
    const double pixel_error_y = detect_y_ + detect_h_ / 2.0 - cy_;
    Eigen::Vector3d horizontal_direction = Eigen::Vector3d::Zero();
    bool direction_valid = false;

    if (std::isfinite(aircraft_.yaw) && std::isfinite(aircraft_.pitch) && std::isfinite(aircraft_.roll) &&
        std::isfinite(config_.focal_length) && config_.focal_length > 0.0) {
        // aircraft_.body_to_ned cached from the latest odometry_sub_ callback (see
        // vision_png_control.hpp) instead of being rebuilt here every 200Hz tick;
        // the isfinite() guards above still mirror the values aircraft_.body_to_ned was built from.
        const Eigen::Vector3d los_ned = aircraft_.body_to_ned * R_c2b_ *
            Eigen::Vector3d(pixel_error_x, pixel_error_y, config_.focal_length);
        const double horizontal_norm = std::hypot(los_ned.x(), los_ned.y());
        direction_valid = los_ned.allFinite() && horizontal_norm > 1e-6;
        if (direction_valid) {
            horizontal_direction = Eigen::Vector3d(
                los_ned.x() / horizontal_norm, los_ned.y() / horizontal_norm, 0.0);
        }
    }
    const bool valid_bbox = state_ == MissionStage::INTERCEPT && direction_valid &&
        aircraft_.horizontal_velocity_valid && aircraft_.vertical_velocity_valid &&
        sample_age >= 0.0 && sample_age <= config_.detection_timeout_seconds &&
        detect_w_ > 0 && detect_h_ > 0 && detect_x_ > 0 && detect_y_ > 0 &&
        static_cast<double>(detect_x_) + detect_w_ < config_.image_width &&
        static_cast<double>(detect_y_) + detect_h_ < config_.image_height;

    double desired_speed = 0.0;
    double width_error = 0.0;
    double alignment_error = 0.0;
    double alignment_scale = 0.0;
    const bool was_stop_latched = approach_stop_latched_;
    const char *transition_reason = "invalid_tracking";
    double resume_elapsed = 0.0;
    if (valid_bbox) {
        const bool new_sample = !approach_width_initialized_ ||
            detection_received_at_ != approach_sample_at_;
        if (!approach_width_initialized_) {
            approach_resume_pending_ = false;
            approach_filtered_width_ = detect_w_;
            approach_width_initialized_ = true;
            approach_sample_at_ = detection_received_at_;
        } else if (detection_received_at_ != approach_sample_at_) {
            const double sample_dt = std::chrono::duration<double>(
                detection_received_at_ - approach_sample_at_).count();
            if (sample_dt <= 0.0 || sample_dt > config_.detection_timeout_seconds) {
                approach_filtered_width_ = detect_w_;
                approach_resume_pending_ = false;
            } else {
                const double alpha = sample_dt / (0.15 + sample_dt);
                approach_filtered_width_ += alpha * (detect_w_ - approach_filtered_width_);
            }
            approach_sample_at_ = detection_received_at_;
        }
        if (approach_filtered_width_ >= config_.approach_bbox_width) {
            approach_stop_latched_ = true;
            approach_resume_pending_ = false;
            transition_reason = "stop_width_reached";
        } else if (approach_filtered_width_ >= config_.approach_resume_bbox_width) {
            approach_resume_pending_ = false;
        } else if (approach_stop_latched_ && new_sample) {
            if (!approach_resume_pending_) {
                approach_resume_started_at_ = detection_received_at_;
                approach_resume_pending_ = true;
            }
            resume_elapsed = std::chrono::duration<double>(
                detection_received_at_ - approach_resume_started_at_).count();
            if (resume_elapsed >= config_.approach_resume_confirm_seconds) {
                approach_stop_latched_ = false;
                approach_resume_pending_ = false;
                transition_reason = "resume_width_confirmed";
            }
        }
        width_error = (config_.approach_bbox_width - approach_filtered_width_) / config_.approach_bbox_width;
        const double image_angle_x = std::atan2(pixel_error_x, config_.focal_length);
        const double image_angle_y = std::atan2(pixel_error_y, config_.focal_length);

        // // 
        const double los_heading = std::atan2(horizontal_direction.y(), horizontal_direction.x());
        const double heading_error = std::remainder(los_heading - aircraft_.yaw, 2.0 * M_PI);
        alignment_error = std::max({std::abs(image_angle_x),
            std::abs(image_angle_y), std::abs(heading_error)});
        alignment_scale = std::clamp(
            (M_PI / 6.0 - alignment_error) / (M_PI / 12.0), 0.0, 1.0);
        desired_speed = std::clamp(config_.approach_kp * width_error, 0.0, config_.approach_max_speed)
            * alignment_scale;
        if (approach_stop_latched_) {
            desired_speed = 0.0;
        }
    } else {
        approach_width_initialized_ = false;
        approach_stop_latched_ = true;
        approach_resume_pending_ = false;
    }

    if (approach_resume_pending_) {
        resume_elapsed = std::chrono::duration<double>(
            detection_received_at_ - approach_resume_started_at_).count();
    }
    if (was_stop_latched != approach_stop_latched_) 
    {
        #ifndef USE_ROM_STYLE_LOG
        logger_.logf(LogLevel::Info, rom_color::BrightRed,
            "[APPROACH TRANSITION] stop_latched=%s reason=%s steady_s=%.3f "
            "bbox=%d filtered=%.2f resume_elapsed=%.3f confirm_seconds=%.3f",
            approach_stop_latched_ ? "true" : "false", transition_reason,
            std::chrono::duration<double>(now.time_since_epoch()).count(),
            detect_w_, approach_filtered_width_, resume_elapsed, config_.approach_resume_confirm_seconds);
        #else
        rom_cout << rom_color::BrightRed << "[APPROACH TRANSITION] stop_latched="
                  << (approach_stop_latched_ ? "true" : "false") << " reason=" << transition_reason
                  << " bbox=" << detect_w_ << " filtered=" << approach_filtered_width_
                  << " resume_elapsed=" << resume_elapsed;
        #endif
    }
    const double max_change = config_.approach_accel_limit * dt;
    if (!valid_bbox || alignment_scale == 0.0 || approach_stop_latched_) {
        approach_speed_ = 0.0;
    } else {
        approach_speed_ += std::clamp(desired_speed - approach_speed_, -max_change, max_change);
    }
    const double shadow_vx = approach_speed_ * horizontal_direction.x();
    const double shadow_vy = approach_speed_ * horizontal_direction.y();
    approach_vx_ = config_.approach_enabled ? static_cast<float>(shadow_vx) : 0.0f;
    approach_vy_ = config_.approach_enabled ? static_cast<float>(shadow_vy) : 0.0f;

    #ifndef USE_ROM_STYLE_LOG
    logger_.log_throttled(LogLevel::Info, {"guidance", "approach_status"}, 500,
        "[APPROACH %s] valid=%s bbox=%d filtered=%.1f desired=%.1f error=%.3f "
        "resume_width=%.1f stop_latched=%s resume_pending=%s resume_elapsed=%.3f "
        "align_deg=%.1f align_scale=%.3f target_speed=%.3f "
        "speed=%.3f vx_N=%.3f vy_E=%.3f output_vx_N=%.3f output_vy_E=%.3f",
        config_.approach_enabled ? "ENABLED" : "SHADOW ONLY", valid_bbox ? "true" : "false",
        detect_w_, approach_filtered_width_, config_.approach_bbox_width,
        width_error, config_.approach_resume_bbox_width, approach_stop_latched_ ? "true" : "false",
        approach_resume_pending_ ? "true" : "false", resume_elapsed,
        alignment_error * 180.0 / M_PI, alignment_scale, desired_speed,
        approach_speed_, shadow_vx, shadow_vy, approach_vx_, approach_vy_);
    #else
    
    #endif
}

#if USE_BC_V2_DEBIASED_MODEL
void VisionPNGGuidance::reset_bc_v2_state()
{
    bc_v2_observer_.reset();
    bc_v2_features_ = {};
    bc_v2_raw_output_ = {};
    bc_v2_command_ = {};
    bc_v2_last_sample_at_ = {};
    bc_v2_has_sample_ = false;
    bc_v2_ready_ = false;
    bc_v2_filter_initialized_ = false;
}

void VisionPNGGuidance::update_bc_v2_shadow()
{
    if (!has_detection_sample_) {
        reset_bc_v2_state();
        return;
    }
    if (bc_v2_has_sample_ && detection_sample_at_ == bc_v2_last_sample_at_) {
        return;
    }
    if (!aircraft_.horizontal_velocity_valid || !aircraft_.vertical_velocity_valid ||
        detect_w_ <= 0 || !std::isfinite(ex_) || !std::isfinite(ey_) ||
        !std::isfinite(LOS_angle_v_) || !std::isfinite(LOS_angle_z_))
    {
        reset_bc_v2_state();
        return;
    }

    const double sample_time_s = std::chrono::duration<double>(
        detection_sample_at_.time_since_epoch()).count();
    bc_v2_last_sample_at_ = detection_sample_at_;
    bc_v2_has_sample_ = true;

    bc_v2_ready_ = bc_v2_observer_.update(
        sample_time_s,
        true,
        ex_,
        ey_,
        LOS_angle_v_,
        LOS_angle_z_,
        static_cast<double>(detect_w_),
        aircraft_.vx,
        aircraft_.vy,
        aircraft_.vz,
        bc_v2_features_,
        cx_,
        cy_);

    if (!bc_v2_ready_) {
        bc_v2_filter_initialized_ = false;
        bc_v2_raw_output_ = {};
        bc_v2_command_ = {};
        return;
    }

    bc_v2_raw_output_ = rom_bc_v2::forward_physical(bc_v2_features_);
    const double max_horiz_speed = std::min(config_.speed_cmd, 8.0);
    const double max_yaw_rate = config_.yaw_rate_limit > 0.0
        ? std::clamp(config_.yaw_rate_limit, 0.5, 1.8)
        : 1.8;
    const auto limited = rom_bc_v2::limit_physical_action(
        bc_v2_raw_output_, max_horiz_speed, 3.0, max_yaw_rate);
    if (!bc_v2_filter_initialized_) {
        bc_v2_command_ = limited;
        bc_v2_filter_initialized_ = true;
    } else {
        constexpr float alpha = 0.5f;
        for (std::size_t i = 0; i < rom_bc_v2::kOutputDim; ++i) {
            bc_v2_command_[i] += alpha * (limited[i] - bc_v2_command_[i]);
        }
    }
    #ifdef USE_ROM_STYLE_LOG
        rom_cout << rom_color::BrightGreen << "[BC_V2 " << (bc_v2_active_control_ ? "ACTIVE" : "SHADOW")
                 << "] cmd=(" << bc_v2_command_[0] << "," << bc_v2_command_[1] << ","
                 << bc_v2_command_[2] << "," << bc_v2_command_[3] << ")";
    #endif
}
#endif

void VisionPNGGuidance::guidance_tick()
{
    aircraft_ = follower_->state();
    // Only update during the intercept phase while the target is valid
    if (state_ != MissionStage::INTERCEPT) {
        reset_yaw_pid();
        yaw_detect_fresh_ = false;
#if USE_BC_V2_DEBIASED_MODEL
        reset_bc_v2_state();
#endif
        return;
    }
    if (reset_if_detection_timed_out()) return;
    if (detect_w_ == -1) return;  // Target lost, do not update

    // ---------- Step 1: Compute pixel error (relative to image center) ----------
    // cx_/cy_ cached once in the constructor (config_.image_width/config_.image_height never change
    // after startup — see vision_png_control.hpp).
    // bbox center
    ex_ = (detect_x_ + detect_w_ / 2.0) - cx_;
    ey_ = (detect_y_ + detect_h_ / 2.0) - cy_;
    update_yaw_pid();

    // ---------- Step 2: Build the camera-frame LOS vector ----------
    // Consistent with los_control.cpp: Nt = [ex, ey, focal]
    Eigen::Vector3d Nt(ex_, ey_, config_.focal_length);

    // ---------- Step 3: Camera frame → body frame → NED frame ----------
    // Body→NED rotation matrix aircraft_.body_to_ned is cached from the latest odometry_sub_ callback
    // (see aircraft_.roll/aircraft_.pitch/aircraft_.yaw comment in vision_png_control.hpp) instead of being rebuilt
    // here every 40Hz tick.
    Eigen::Vector3d Nt_ned = aircraft_.body_to_ned * R_c2b_ * Nt;

    // ---------- Step 4: Compute the LOS angles ----------
    double rxy = std::sqrt(Nt_ned(0)*Nt_ned(0) + Nt_ned(1)*Nt_ned(1));
    LOS_angle_v_ = std::atan2(Nt_ned(2), rxy);       // Elevation (NED Down is positive)
    LOS_angle_z_ = std::atan2(Nt_ned(0), Nt_ned(1)); // Azimuth (North=0, clockwise)

    #ifdef USE_Vz_PID_CONTROL
        if (detect_fresh_)
        {
            const double dt = vz_pid_.initialized()
                ? std::chrono::duration<double>(
                    detection_sample_at_ - vz_pid_previous_sample_at_).count()
                : 0.0;
            // Use attitude-compensated NED vertical error (in pixel-equivalent units)
            // so forward pitch during high-speed pursuit does not trick Vz PID into climbing.
            const double ey_ned = (rxy > 1e-6 && std::isfinite(LOS_angle_v_))
                ? (config_.focal_length * std::tan(LOS_angle_v_))
                : ey_;
            rom_vz_ = vz_inline_pid_control(ey_ned, dt);
            vz_pid_previous_sample_at_ = detection_sample_at_;
            vz_pid_last_consumed_sample_at_ = detection_sample_at_;
            vz_pid_has_consumed_sample_ = true;
            detect_fresh_ = false;
        }
    #endif

#if USE_BC_V2_DEBIASED_MODEL
    update_bc_v2_shadow();
#endif

    // Check whether this is identical to the previous frame (duplicate frame, skip update)
    if (png_initialized_ && Nt_ned == last_LOS_vec_) return;
    last_LOS_vec_ = Nt_ned;
    LOS_vec_      = Nt_ned;

    // ---------- Step 5: PNG angle update ----------
    // Only initialize on first entry, using the LOS direction as the initial velocity direction
    // Note: no longer resets on |V|<1, avoiding repeated resets during acceleration that cause oscillation
    Eigen::Vector3d Vt(aircraft_.vx, aircraft_.vy, aircraft_.vz);
    if (!png_initialized_) 
    {
        d_v_angle_v_      = LOS_angle_v_;
        d_v_angle_z_      = LOS_angle_z_;
        last_LOS_angle_v_ = LOS_angle_v_;
        last_LOS_angle_z_ = LOS_angle_z_;
        last_v_angle_v_   = LOS_angle_v_;  // Use LOS rather than measured velocity, avoiding take-off residual contamination
        last_v_angle_z_   = LOS_angle_z_;
        png_initialized_  = true;
#ifdef PNG_DEBUG
    #ifndef USE_ROM_STYLE_LOG
        logger_.logf(LogLevel::Info, rom_color::DarkBlue,
            "[PNG init] LOS(v=%.3f z=%.3f) |V|=%.2f",
            LOS_angle_v_, LOS_angle_z_, Vt.norm());
    #else
        rom_cout << rom_color::DarkBlue << "[PNG init] LOS(v=" << LOS_angle_v_ << " z=" << LOS_angle_z_ << ") |V|=" << Vt.norm();
    #endif  
#endif
        return;
    }

    double diff_LOS_v = LOS_angle_v_ - last_LOS_angle_v_;
    double diff_LOS_z = LOS_angle_z_ - last_LOS_angle_z_;

    // Handle ±π wraparound (azimuth)
    while (diff_LOS_z >  M_PI) diff_LOS_z -= 2.0 * M_PI;
    while (diff_LOS_z < -M_PI) diff_LOS_z += 2.0 * M_PI;
    while (diff_LOS_v >  M_PI) diff_LOS_v -= 2.0 * M_PI;
    while (diff_LOS_v < -M_PI) diff_LOS_v += 2.0 * M_PI;

    // Only update when the LOS change is large enough (consistent with the 0.02 threshold in los_control.cpp)
    if (std::abs(diff_LOS_v) > 0.02 || std::abs(diff_LOS_z) > 0.02) 
    {
        // Use the "previous desired velocity angle" as the PNG reference (fixed reference, does not follow measured velocity)
        // This avoids PNG divergence caused by measured-velocity lag during large maneuvers
        // PNG core formula (fully consistent with los_control.cpp)
        d_v_angle_v_ = config_.kv * diff_LOS_v + last_v_angle_v_;
        d_v_angle_z_ = config_.kz * diff_LOS_z + last_v_angle_z_;

        // Bound lead angles relative to current LOS so circular target motion
        // cannot wind up d_v_angle_z_/d_v_angle_v_ > 90° away from the target.
        constexpr double max_lead_z = M_PI / 4.0;  // ±45° max azimuth lead
        constexpr double max_lead_v = M_PI / 6.0;  // ±30° max elevation lead
        const double lead_z = std::clamp(
            std::remainder(d_v_angle_z_ - LOS_angle_z_, 2.0 * M_PI),
            -max_lead_z, max_lead_z);
        d_v_angle_z_ = LOS_angle_z_ + lead_z;

        const double lead_v = std::clamp(
            std::remainder(d_v_angle_v_ - LOS_angle_v_, 2.0 * M_PI),
            -max_lead_v, max_lead_v);
        d_v_angle_v_ = LOS_angle_v_ + lead_v;

        // Clamp the elevation range (±45°) to prevent excessive elevation from sending the drone rocketing upward
        d_v_angle_v_ = std::max(-M_PI/4.0, std::min(M_PI/4.0, d_v_angle_v_));

        // Update last_v_angle to this cycle's desired value (iterative update)
        last_v_angle_v_ = d_v_angle_v_;
        last_v_angle_z_ = d_v_angle_z_;

        last_LOS_angle_v_ = LOS_angle_v_;
        last_LOS_angle_z_ = LOS_angle_z_;

        #ifdef PNG_DEBUG
            #ifndef USE_ROM_STYLE_LOG
                logger_.logf(LogLevel::Info, rom_color::DarkBlue,
                    "[PNG] ex=%.1f ey=%.1f | LOS(v=%.3f z=%.3f) | dLOS(v=%.3f z=%.3f) | dV(v=%.3f z=%.3f)",
                    ex_, ey_,
                    LOS_angle_v_, LOS_angle_z_,
                    diff_LOS_v, diff_LOS_z,
                    d_v_angle_v_, d_v_angle_z_);
            #else
                rom_cout << rom_color::DarkBlue << "[PNG] ex=" << ex_ << " ey=" << ey_ << " | LOS(v=" << LOS_angle_v_ << " z=" << LOS_angle_z_ << ") | dLOS(v=" << diff_LOS_v << " z=" << diff_LOS_z << ") | dV(v=" << d_v_angle_v_ << " z=" << d_v_angle_z_ << ")";
            #endif
        #endif
    }

    // ---------- Step 7: Publish the /vpng_data debug topic ----------
    follower_->publish_telemetry(telemetry());
}

void VisionPNGGuidance::handle_takeoff()
{
    if (follower_->prepare()) {
        enter_searching();
    }
}

void VisionPNGGuidance::enter_searching()
{
    searching_started_ = follower_->steady_now();
    return_home_commanded_ = false;
    return_home_counter_ = 0;
    state_ = MissionStage::SEARCHING;
}

void VisionPNGGuidance::enter_track_lost()
{
    detect_w_ = -1;
    detect_fresh_ = false;
    yaw_detect_fresh_ = false;
#if USE_BC_V2_DEBIASED_MODEL
    reset_bc_v2_state();
#endif
    track_lost_started_ = follower_->steady_now();
    return_home_commanded_ = false;
    return_home_counter_ = 0;
    state_ = MissionStage::TRACK_LOST;
}

void VisionPNGGuidance::enter_return_home()
{
    detect_w_ = -1;
    detect_fresh_ = false;
    yaw_detect_fresh_ = false;
    d_yaw_ = 0.0;
#if USE_BC_V2_DEBIASED_MODEL
    reset_bc_v2_state();
#endif
    last_published_velocity_command_ = {};
    last_vz_png_only_ = 0.0f;
    last_vz_pid_correction_ = 0.0f;
    has_published_velocity_command_ = false;
    return_home_commanded_ = false;
    return_home_counter_ = 0;
    state_ = MissionStage::RETURN_HOME;
}

void VisionPNGGuidance::handle_searching()
{
    const auto searching_elapsed = follower_->steady_now() - searching_started_;
    const double searching_elapsed_seconds = std::chrono::duration<double>(searching_elapsed).count();
    if (searching_elapsed_seconds >= config_.return_home_timeout_seconds)
    {
        if (follower_->can_return_home())
        {
            logger_.log_throttled(LogLevel::Warning, {"guidance", "return_home"}, 1000,
                rom_color::Default,
                "[SEARCHING] Return-home timeout (%.1f s) reached and return check valid: commanding RTL",
                config_.return_home_timeout_seconds);
            enter_return_home();
            handle_return_home();
            return;
        }
        logger_.log_throttled(LogLevel::Warning, {"guidance", "return_home_invalid"}, 1000,
            rom_color::Default,
            "[SEARCHING] Return-home timeout (%.1f s) reached but EKF/home not valid for RTL: holding position",
            config_.return_home_timeout_seconds);
        follower_->apply(FlightRequest::VelocityHeartbeat);
        publish_velocity_setpoint(0.0f, 0.0f, 0.0f, 0.0f);
        follower_->record(telemetry());
        return;
    }

    const float search_yaw_rate = searching_elapsed > std::chrono::seconds(hover_delay_seconds)
        ? search_yaw_rate_command : 0.0f;
    follower_->apply(FlightRequest::VelocityHeartbeat);

    #ifdef USE_ROM_STYLE_LOG
        rom_cout << rom_color::DarkCyan << "[ Searching ]";
    #endif
    publish_velocity_setpoint(0.0f, 0.0f, 0.0f, search_yaw_rate);

#ifdef TARGET_LOST_DEBUG
    #ifndef USE_ROM_STYLE_LOG
    logger_.log_throttled(LogLevel::Info, {"guidance", "searching"}, 2000,
        rom_color::Red,
        "[INFO throttle=0.5 Hz][SEARCHING] Waiting for vision... detect_w=%d yaw_rate=%.2f",
        detect_w_, search_yaw_rate);
    #else
        rom_cout << rom_color::Red << "[SEARCHING] Waiting for vision... detect_w=" << detect_w_ << " yaw_rate=" << search_yaw_rate;
    #endif
#endif

    // Target detected: the detect_sub_ callback switches to INTERCEPT
}

void VisionPNGGuidance::handle_intercept()
{
    follower_->apply(FlightRequest::Arm);
    follower_->apply(FlightRequest::EnableExternalControl);

#if USE_ELAPSED_TIME_INSTEAD_OF_FRAME_COUNT
    const double measurement_age = std::chrono::duration<double>(
        follower_->steady_now() - detection_sample_at_).count();
    if (has_detection_sample_ && measurement_age >= config_.lost_thresh_timeout_seconds)
    {
        reset_if_detection_timed_out();
        logger_.log_throttled(LogLevel::Warning, {"guidance", "target_lost"}, 500,
            "[INTERCEPT] Detection loss deadline reached, switching to TRACK_LOST");
        enter_track_lost();
        follower_->apply(FlightRequest::VelocityHeartbeat);
        publish_velocity_setpoint(0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }
#endif

    if (reset_if_detection_timed_out())
    {
        follower_->apply(FlightRequest::VelocityHeartbeat);
        publish_velocity_setpoint(0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

#if USE_ELAPSED_TIME_INSTEAD_OF_FRAME_COUNT
    if (detect_w_ == -1 && measurement_age >= config_.coast_thresh_timeout_seconds)
    {
        follower_->apply(FlightRequest::VelocityHeartbeat);
        publish_velocity_setpoint(0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }
#endif

    // PSA ADD: Handle invalid velocity by holding last command for a few ticks before commanding zero
    if (!aircraft_.horizontal_velocity_valid || !aircraft_.vertical_velocity_valid) 
    {
        follower_->apply(FlightRequest::VelocityHeartbeat);
        if (has_published_velocity_command_ && invalid_velocity_ticks_ < invalid_velocity_hold_ticks_) 
        {
            ++invalid_velocity_ticks_;
            publish_velocity_setpoint(
                last_published_velocity_command_[0], last_published_velocity_command_[1],
                last_published_velocity_command_[2], last_published_velocity_command_[3]);
        } 
        else 
        {
            publish_velocity_setpoint(0.0f, 0.0f, 0.0f, 0.0f);
        }


        logger_.log_throttled(LogLevel::Warning, {"guidance", "invalid_velocity"}, 500,
            "[INTERCEPT] Invalid velocity: holding last command for up to 10 ticks, then commanding zero");

        return;
    }
    invalid_velocity_ticks_ = 0;
    // PSA END : Reset invalid velocity ticks when velocity is valid


    // ---- Graded handling of target loss ----
#if USE_ELAPSED_TIME_INSTEAD_OF_FRAME_COUNT
    if (detect_w_ == -1 && measurement_age < config_.coast_thresh_timeout_seconds)
#else
    if (lost_frames_ > 0 && lost_frames_ < config_.coast_thresh)
#endif
    {
        // Brief loss within the selected policy: retain the last velocity and yaw.
        // Keep rotating the nose to search for the target at the same time (keep d_yaw_ unchanged)
        follower_->apply(FlightRequest::VelocityHeartbeat);
        // d_yaw_ ကိုဆက်သိမ်းသင့်/ မသိမ်းသင့် စဥ်းစားရန်
        publish_velocity_setpoint(coast_vx_cmd_, coast_vy_cmd_, coast_vz_cmd_, (float)d_yaw_);
        #ifdef TARGET_LOST_DEBUG
            #ifndef USE_ROM_STYLE_LOG
                logger_.log_throttled(LogLevel::Info, {"guidance", "coasting"}, 100,
                    rom_color::Default,
                    "[INFO throttle=10 Hz][INTERCEPT/COAST] Coasting on inertia lost=%d v=(%.1f,%.1f,%.1f)",
                    lost_frames_, coast_vx_cmd_, coast_vy_cmd_, coast_vz_cmd_);
            #else
                rom_cout << rom_color::Default << "[INTERCEPT/COAST] Coasting on inertia lost=" << lost_frames_ << " v=(" << coast_vx_cmd_ << "," << coast_vy_cmd_ << "," << coast_vz_cmd_ << ")";
            #endif
        #endif
        return;
    }

#if !USE_ELAPSED_TIME_INSTEAD_OF_FRAME_COUNT
    if (lost_frames_ >= config_.lost_thresh)
    {
    #ifndef USE_ROM_STYLE_LOG
        logger_.log_throttled(LogLevel::Warning, {"guidance", "target_lost"}, 500,
            "[INTERCEPT] Target lost for a long time lost_frames=%d, switching to TRACK_LOST", lost_frames_);
    #else
        //rom style log here if needed
    #endif
        enter_track_lost();
        return;
    }
#endif

    // ---- Normal intercept: compute the velocity command ----
    Eigen::Vector3d Vt(aircraft_.vx, aircraft_.vy, aircraft_.vz);
    
    double d_v = Vt.norm() + config_.d_gain;
    d_v = std::min(d_v, config_.speed_cmd);
    d_v = std::max(d_v, 2.0);

    // Compose the NED velocity command from the PNG angles
    float png_vx_cmd = (float)(std::cos(d_v_angle_v_) * std::sin(d_v_angle_z_) * d_v);  // N
    float png_vy_cmd = (float)(std::cos(d_v_angle_v_) * std::cos(d_v_angle_z_) * d_v);  // E
    float png_vz_cmd = (float)(std::sin(d_v_angle_v_) * d_v);                            // D

    // Documented raw PNG vertical clamp (pre-weight). Keeps the PNG branch of the
    // 70/30 complementary blend inside ±5.0 m/s before the ratio weight is applied,
    // so the vertical authority no longer depends implicitly on speed_cmd × sin(45°).
    png_vz_cmd = std::clamp(png_vz_cmd, -5.0f, 5.0f); // raw png_vz_cmd clamp with hard code

    // ROM REMOVE START
    // ---- ey vertical FOV compensation (offsets camera tilting down during forward flight) ----
    // ey < 0: target is above center (image y axis points down, negative = target above)
    //        → UAV needs to climb → NED png_vz_cmd decreases (more negative)
    //        → vz_ey = config_.k_ey * ey_ < 0, once added png_vz_cmd becomes smaller (climb) ✓
    // ey > 0: target is below center → UAV needs to descend → vz_ey > 0, png_vz_cmd increases (descend) ✓
    // Compensation is clamped to ±3 m/s
    // {
    //     float vz_ey = (float)(config_.k_ey * ey_);
    //     vz_ey = std::max(-3.0f, std::min(3.0f, vz_ey));
    //     png_vz_cmd += vz_ey;
    // }
    // ROM REMOVE END

    // ROM ADD THIS PID CORRECTION
    float pid_vz_correction = static_cast<float>(rom_vz_);
    last_vz_png_only_ = static_cast<float>(
        (1.0 - config_.rom_vz_complementary_pid_ratio) * png_vz_cmd);
    last_vz_pid_correction_ = static_cast<float>(
        config_.rom_vz_complementary_pid_ratio * pid_vz_correction);
    float final_vz_cmd = last_vz_png_only_ + last_vz_pid_correction_;

    #ifdef Vz_PID_DEBUG
        rom_cout << rom_color::BrightRed << "[png, correction] png_vz_cmd=" << png_vz_cmd << " pid_vz_correction=" << pid_vz_correction;
        rom_cout << rom_color::BrightRed << "[png] final_vz_cmd=" << final_vz_cmd;
    #endif
    final_vz_cmd = std::clamp(final_vz_cmd, -5.0f, 5.0f);
    // ROM END

    // ---- Horizontal hybrid complementary filter (PNG ↔ approach) ----
    // blend = 0 → pure PNG vx/vy (aggressive, far target)
    // blend = 1 → pure approach vx/vy (gentle terminal creep, stop-latch capable)
    // blend ramps linearly as filtered bbox width grows from
    // config_.hybrid_blend_start_width up to config_.approach_bbox_width, then rate-limits the
    // applied weight so switching never steps the velocity command.
    float final_vx_cmd = png_vx_cmd;
    float final_vy_cmd = png_vy_cmd;
    update_horizontal_approach_shadow();  // refresh approach_vx_/approach_vy_ before blending
    if (config_.approach_enabled) {
        double blend_target = 0.0;
        if (approach_width_initialized_) {
            const double w = approach_filtered_width_;
            blend_target = std::clamp(
                (w - config_.hybrid_blend_start_width) / (config_.approach_bbox_width - config_.hybrid_blend_start_width),
                0.0, 1.0);
        }
        const double blend_step = 0.15;  // ~20 ticks @20Hz for a full PNG↔approach swap
        horizontal_blend_ += std::clamp(blend_target - horizontal_blend_, -blend_step, blend_step);
        final_vx_cmd = (float)((1.0 - horizontal_blend_) * png_vx_cmd + horizontal_blend_ * approach_vx_);
        final_vy_cmd = (float)((1.0 - horizontal_blend_) * png_vy_cmd + horizontal_blend_ * approach_vy_);
    } else {
        horizontal_blend_ = 0.0;
    }
    #ifdef USE_ROM_STYLE_LOG
        rom_cout << rom_color::BrightMagenta << "[HYBRID] blend=" << horizontal_blend_
                  << " png=(" << png_vx_cmd << "," << png_vy_cmd << ")"
                  << " approach=(" << approach_vx_ << "," << approach_vy_ << ")"
                  << " final=(" << final_vx_cmd << "," << final_vy_cmd << ")";
    #endif

    // Cache the velocity for inertial coast-flight use
    float applied_yaw_rate = static_cast<float>(d_yaw_);
#if USE_BC_V2_DEBIASED_MODEL
    if (bc_v2_active_control_ && bc_v2_ready_) {
        final_vx_cmd = bc_v2_command_[0];
        final_vy_cmd = bc_v2_command_[1];
        final_vz_cmd = bc_v2_command_[2];
        applied_yaw_rate = bc_v2_command_[3];
        last_vz_png_only_ = final_vz_cmd;
        last_vz_pid_correction_ = 0.0f;
    }
#endif
    coast_vx_cmd_  = final_vx_cmd;
    coast_vy_cmd_  = final_vy_cmd;
    coast_vz_cmd_  = final_vz_cmd;
    coast_yaw_ = applied_yaw_rate;

    follower_->apply(FlightRequest::VelocityHeartbeat);
    #ifdef USE_ROM_STYLE_LOG
        rom_cout << rom_color::DarkCyan << "[ Calculated ]";
    #endif
    publish_velocity_setpoint(final_vx_cmd, final_vy_cmd, final_vz_cmd, applied_yaw_rate);

    // ---- Statistics: update closest-approach distance + hit detection ----
    if (follower_->record(telemetry()).completed) {
        detect_w_ = -1;
        detect_fresh_ = false;
        yaw_detect_fresh_ = false;
        state_ = MissionStage::DONE;
    }

#ifdef INTERCEPT_DEBUG
    #ifndef USE_ROM_STYLE_LOG
    logger_.log_throttled(LogLevel::Info, {"guidance", "intercept"}, 200,
        rom_color::Default,
        "[INFO throttle=5 Hz][INTERCEPT] v_cmd=(%.2f,%.2f,%.2f) yaw_r=%.3f |V|=%.2f ex=%.0f ey=%.0f",
        png_vx_cmd, png_vy_cmd, png_vz_cmd, d_yaw_, Vt.norm(), ex_, ey_);
    #else
        rom_cout << rom_color::Default << "[INTERCEPT] v_cmd=(" << png_vx_cmd << "," << png_vy_cmd << "," << png_vz_cmd << ") yaw_r=" << d_yaw_ << " |V|=" << Vt.norm() << " ex=" << ex_ << " ey=" << ey_;
    #endif
#endif
}

void VisionPNGGuidance::handle_track_lost()
{
    const double track_lost_elapsed = std::chrono::duration<double>(
        follower_->steady_now() - track_lost_started_).count();
    if (track_lost_elapsed >= config_.return_home_timeout_seconds)
    {
        if (follower_->can_return_home())
        {
            logger_.log_throttled(LogLevel::Warning, {"guidance", "return_home"}, 1000,
                rom_color::Default,
                "[TRACK_LOST] Return-home timeout (%.1f s) reached and return check valid: commanding RTL",
                config_.return_home_timeout_seconds);
            enter_return_home();
            handle_return_home();
            return;
        }
        // PSA ADD
        // If return home is not possible, we may want to add additional handling here, 
        // such as computing return path by companion computer.

        // PSA END`
        logger_.log_throttled(LogLevel::Warning, {"guidance", "return_home_invalid"}, 1000,
            rom_color::Default,
            "[TRACK_LOST] Return-home timeout (%.1f s) reached but EKF/home not valid for RTL: holding position",
            config_.return_home_timeout_seconds);
        follower_->apply(FlightRequest::VelocityHeartbeat);
        publish_velocity_setpoint(0.0f, 0.0f, 0.0f, 0.0f);
        follower_->record(telemetry());
        return;
    }

    // Brake: send zero translation velocity and rotate toward the side where the target last exited
    const float search_yaw = static_cast<float>(
        track_lost_search_sign_ * config_.re_search_yaw_rate_after_track_lost_rd_per_sec);
    follower_->apply(FlightRequest::VelocityHeartbeat);
    #ifdef USE_ROM_STYLE_LOG
        rom_cout << rom_color::DarkCyan << "[ TrackLost ]";
    #endif
    publish_velocity_setpoint(0.0f, 0.0f, 0.0f, search_yaw);
    follower_->record(telemetry());

#ifndef USE_ROM_STYLE_LOG
    logger_.log_throttled(LogLevel::Warning, {"guidance", "track_lost"}, 1000,
        rom_color::Default,
        "[TRACK_LOST] Decelerating and rotating to search for the target... yaw_rate=%.2f", search_yaw);
    #else
        rom_cout << rom_color::Default << "[TRACK_LOST] Decelerating and rotating to search for the target... yaw_rate=" << search_yaw;
    #endif
        

    // Recovery logic is handled by the detect_sub_ callback
}

void VisionPNGGuidance::handle_return_home()
{
    if (!aircraft_.return_home_active)
    {
        if (!return_home_commanded_ || return_home_counter_ % 200 == 0)
        {
            follower_->apply(FlightRequest::ReturnHome);
            return_home_commanded_ = true;
        }
        return_home_counter_++;
    }
    follower_->record(telemetry());
}

void VisionPNGGuidance::handle_done()
{
    follower_->apply(FlightRequest::VelocityHeartbeat);
    #ifdef USE_ROM_STYLE_LOG
        rom_cout << rom_color::DarkCyan << "[ Done ]";
    #endif
    publish_velocity_setpoint(0.0f, 0.0f, 0.0f, 0.0f);
#ifdef INTERCEPT_DEBUG
    #ifndef USE_ROM_STYLE_LOG
    logger_.log_once(LogLevel::Info, {"guidance", "done"}, rom_color::Magenta, "★★★ Intercept complete, hovering ★★★");
    #else
        rom_cout << rom_color::Magenta << "★★★ Intercept complete, hovering ★★★";
    #endif
#endif
}

void VisionPNGGuidance::publish_velocity_setpoint(float requested_vx, float requested_vy, float requested_vz, float requested_yaw_rate)
{
    const bool detection_timed_out =
        state_ == MissionStage::INTERCEPT && reset_if_detection_timed_out();

    if (detection_timed_out)
    {
        follower_->apply(VelocityYawRateSetpoint{0.0f, 0.0f, 0.0f, 0.0f});
        return;
    }

    // Only yaw rate is PID controlled; force vertical velocity to zero.
    // requested_vz = 0.0f;

#if !ENABLE_HORIZONTAL_OUTPUT
    requested_vx = 0.0f;
    requested_vy = 0.0f;
#endif

    last_published_velocity_command_ =
        {requested_vx, requested_vy, requested_vz, requested_yaw_rate};
    has_published_velocity_command_ = true;

    follower_->apply(VelocityYawRateSetpoint{requested_vx, requested_vy, requested_vz, requested_yaw_rate});
    

    #ifdef PUBLISHED_DEBUG
        #ifndef USE_ROM_STYLE_LOG
            logger_.logf(LogLevel::Info, rom_color::Cyan,
                            "vx, vy, vz: %.7f %.7f %.7f",
                            requested_vx, requested_vy, requested_vz);
        #else
            rom_cout << rom_color::Cyan << "[ PUBLISHED ] vx=" << requested_vx << " vy=" << requested_vy << " vz=" << requested_vz << " yaw_rate=" << requested_yaw_rate;
        #endif
    #endif
}

void VisionPNGGuidance::reset_yaw_pid()
{
    yaw_pid_.reset();
    yaw_pid_previous_sample_at_ = {};
}

void VisionPNGGuidance::update_yaw_pid()
{
    constexpr double output_limit = 1.0;
    const double max_sample_gap =
        std::chrono::duration<double>(detection_timeout_).count();

    const double sample_age = std::chrono::duration<double>(
        follower_->steady_now() - detection_sample_at_).count();

    if (!std::isfinite(ex_) || !has_detection_sample_ ||
        sample_age < 0.0 || sample_age > max_sample_gap) {
        reset_yaw_pid();
        yaw_detect_fresh_ = false;
        d_yaw_ = 0.0;
        return;
    }
    if (!yaw_detect_fresh_) return;
    yaw_detect_fresh_ = false;

    const double dt = std::chrono::duration<double>(
        detection_sample_at_ - yaw_pid_previous_sample_at_).count();
        
    d_yaw_ = yaw_pid_.update(ex_, dt, output_limit);
    yaw_pid_previous_sample_at_ = detection_sample_at_;
    yaw_pid_last_consumed_sample_at_ = detection_sample_at_;
    yaw_pid_has_consumed_sample_ = true;
    
    #ifdef Yaw_PID_DEBUG
        rom_cout << rom_color::DarkBlue << "[ Yaw PID ] dt=" << dt
                  << " raw=" << yaw_pid_.raw_output() << " d_yaw=" << d_yaw_;
    #endif
}

float VisionPNGGuidance::vz_inline_pid_control(double error_y, double dt)
{
    if (!std::isfinite(error_y)) 
    {
        vz_pid_.reset();
        rom_vz_ = 0.0;
        return 0.0f;
    }

    constexpr double correction_limit = 3.0; // raw PID output limit for vz
    rom_vz_ = vz_pid_.update(error_y, dt, correction_limit);

    #ifdef Vz_PID_DEBUG
    rom_cout << rom_color::BrightRed << "[i-term] i_term = " << config_.control_vz_gain_2_i * vz_pid_.integral();
    #endif

    return static_cast<float>(rom_vz_);
}

GuidanceTelemetry VisionPNGGuidance::telemetry() const
{
    const Eigen::Vector3d velocity(aircraft_.vx, aircraft_.vy, aircraft_.vz);
    GuidanceTelemetry data;
    data.aircraft = aircraft_;
    data.stage = state_;
    data.bbox_width = detect_w_;
    data.desired_angle_v = d_v_angle_v_;
    data.desired_angle_z = d_v_angle_z_;
    data.los_angle_v = LOS_angle_v_;
    data.los_angle_z = LOS_angle_z_;
    data.diff_los_angle_v = LOS_angle_v_ - last_LOS_angle_v_;
    data.diff_los_angle_z = LOS_angle_z_ - last_LOS_angle_z_;
    data.velocity_angle_v = std::atan2(velocity(2),
        std::sqrt(velocity(1) * velocity(1) + velocity(0) * velocity(0)));
    data.velocity_angle_z = std::atan2(velocity(0), velocity(1));
    data.ex = ex_;
    data.ey = ey_;
    data.pixel_u = detect_x_ + detect_w_ / 2.0;
    data.pixel_v = detect_y_ + detect_h_ / 2.0;
    data.speed = velocity.norm();
    data.requested_vx = last_published_velocity_command_[0];
    data.requested_vy = last_published_velocity_command_[1];
    data.requested_vz = last_published_velocity_command_[2];
    data.vz_png_only = last_vz_png_only_;
    data.vz_pid_correction = last_vz_pid_correction_;
    data.yaw_rate = has_published_velocity_command_
        ? static_cast<double>(last_published_velocity_command_[3])
        : d_yaw_;
    return data;
}

}  // namespace ct_follower_png
