/* n6d_velocity_controller_node.cpp
 *
 * ROS 2 node that consumes nav6d planner paths and applies a 6-DOF PD controller to generate
 * body-frame twist commands for the cmd_bridge. Parameters configure topics, gains, clamps, and
 * tolerances; subscriptions cache the latest path/pose/IMU samples; control_step projects the
 * robot onto the path, computes a carrot pose, runs PD, clamps, and publishes the twist.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"

namespace {
// Small helper struct used to store per-axis PD gains.
struct PDGains {
    double kp[3]{0.0, 0.0, 0.0};
    double kd[3]{0.0, 0.0, 0.0};
};

// Clamp helper that keeps each axis of a vector inside the provided limits.
tf2::Vector3 clamp_each(const tf2::Vector3& value, const tf2::Vector3& limits) {
    auto clamp_axis = [](double v, double limit) {
        return std::copysign(std::min(std::abs(v), limit), v);
    };
    return {clamp_axis(value.x(), limits.x()), clamp_axis(value.y(), limits.y()),
            clamp_axis(value.z(), limits.z())};
}
}  // namespace

// Converts a nav_msgs/Path into twist commands by subscribing to planner/pose/IMU topics,
// projecting onto the path, running a PD controller, clamping the output, and publishing body-frame
// twists.
class N6dVelocityController : public rclcpp::Node {
   public:
    // Constructor wires up parameters, ROS interfaces, and timer scheduling.
    N6dVelocityController() : rclcpp::Node("n6d_velocity_controller") {
        declare_parameters();
        init_subscriptions();
        init_publishers();
        init_timer();

        RCLCPP_INFO(get_logger(),
                    "n6d_velocity_controller ready. path=%s pose=%s imu=%s -> velocity=%s (frame=body)",
                    path_topic_.c_str(), pose_topic_.c_str(), imu_topic_.c_str(),
                    cmd_velocity_topic_.c_str());
    }

   private:
    // --- Parameter loading -------------------------------------------------
    // Load topics, gains, limits, and other tuning knobs so they can be overridden via YAML.
    void declare_parameters() {
        // Topic wiring
        path_topic_ = declare_parameter<std::string>("path_topic", "/nav6d/planner/path");
        pose_topic_ = declare_parameter<std::string>("pose_topic", "/space_cobot/pose");
        imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
        cmd_velocity_topic_ =
            declare_parameter<std::string>("cmd_velocity_topic", "/space_cobot/cmd_vel");
        (void)declare_parameter<std::string>("command_frame", "body");  // legacy, ignored
        use_goal_orientation_ = declare_parameter<bool>("use_goal_orientation", false);

        // Control knobs
        control_rate_hz_ = declare_parameter("control_rate_hz", 50.0);
        lookahead_distance_ = declare_parameter("lookahead_distance", 0.8);
        path_reacquire_period_ = declare_parameter("path_reacquire_period", 0.5);
        feedforward_speed_ = declare_parameter("feedforward_speed", 0.0);
        approach_slowdown_distance_ = declare_parameter("approach_slowdown_distance", 1.0);
        vel_alpha_ = declare_parameter("velocity_ema_alpha", 0.6);
        pos_tolerance_ = declare_parameter("pos_tolerance", 0.05);
        yaw_tolerance_rad_ = declare_parameter("yaw_tolerance_rad", 0.08);
        debug_enabled_ = declare_parameter("debug_enabled", false);
        debug_speed_topic_ = declare_parameter<std::string>(
            "debug_speed_topic", "/nav6d/velocity_controller/debug/linear_speed");
        debug_projected_pose_topic_ = declare_parameter<std::string>(
            "debug_projected_pose_topic", "/nav6d/velocity_controller/debug/path_projection");
        debug_target_pose_topic_ = declare_parameter<std::string>(
            "debug_target_pose_topic", "/nav6d/velocity_controller/debug/carrot_pose");
        debug_error_topic_ = declare_parameter<std::string>(
            "debug_error_topic", "/nav6d/velocity_controller/debug/control_error");

        // Linear (xyz) gains
        const std::vector<double> kp_lin =
            declare_parameter<std::vector<double>>("kp_linear", {1.2, 1.2, 1.5});
        const std::vector<double> kd_lin =
            declare_parameter<std::vector<double>>("kd_linear", {0.6, 0.6, 0.8});
        // Angular (rpy) gains
        const std::vector<double> kp_ang =
            declare_parameter<std::vector<double>>("kp_angular", {2.0, 2.0, 2.0});
        const std::vector<double> kd_ang =
            declare_parameter<std::vector<double>>("kd_angular", {0.2, 0.2, 0.3});

        // Saturation limits (per axis)
        const std::vector<double> max_velocity =
            declare_parameter<std::vector<double>>("max_linear_velocity_xyz", {0.6, 0.6, 0.4});
        const std::vector<double> max_torque =
            declare_parameter<std::vector<double>>("max_angular_velocity_rpy", {0.6, 0.6, 0.6});

        for (int axis = 0; axis < 3; ++axis) {
            gains_pos_.kp[axis] = axis < static_cast<int>(kp_lin.size()) ? kp_lin[axis] : 0.0;
            gains_pos_.kd[axis] = axis < static_cast<int>(kd_lin.size()) ? kd_lin[axis] : 0.0;
            gains_att_.kp[axis] = axis < static_cast<int>(kp_ang.size()) ? kp_ang[axis] : 0.0;
            gains_att_.kd[axis] = axis < static_cast<int>(kd_ang.size()) ? kd_ang[axis] : 0.0;
        }

        auto pick_limit = [](const std::vector<double>& src, size_t idx) {
            return idx < src.size() ? src[idx] : 0.0;
        };
        max_linear_cmd_body_ = tf2::Vector3(
            pick_limit(max_velocity, 0), pick_limit(max_velocity, 1), pick_limit(max_velocity, 2));
        max_angular_cmd_body_ = tf2::Vector3(pick_limit(max_torque, 0), pick_limit(max_torque, 1),
                                             pick_limit(max_torque, 2));
    }

    // --- ROS interface wiring ----------------------------------------------
    // Subscribe to planner path, robot pose, and IMU topics.
    void init_subscriptions() {
        // Subscription callbacks only cache the latest messages; the control loop samples them at
        // its own rate so callbacks remain lightweight.
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            path_topic_, 10,
            std::bind(&N6dVelocityController::handle_path, this, std::placeholders::_1));
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, rclcpp::SensorDataQoS(),
            std::bind(&N6dVelocityController::handle_pose, this, std::placeholders::_1));
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS(),
            std::bind(&N6dVelocityController::handle_imu, this, std::placeholders::_1));
    }

    // Create command and optional debug publishers.
    void init_publishers() {
        // Pre-create command/debug publishers; debug topics are optional and only advertised when
        // enabled to avoid cluttering the graph.
        velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_velocity_topic_, 10);
        if (debug_enabled_) {
            if (!debug_speed_topic_.empty()) {
                speed_pub_ = create_publisher<std_msgs::msg::Float32>(debug_speed_topic_, 10);
            }
            if (!debug_projected_pose_topic_.empty()) {
                debug_projected_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
                    debug_projected_pose_topic_, 10);
            }
            debug_target_pose_pub_ =
                create_publisher<geometry_msgs::msg::PoseStamped>(debug_target_pose_topic_, 10);
            debug_error_pub_ =
                create_publisher<geometry_msgs::msg::TwistStamped>(debug_error_topic_, 10);
        }
    }

    // Start the wall timer that drives the control loop.
    void init_timer() {
        // Timer drives the control loop at the configured rate (clamped to >=1 Hz).
        const double clamped_rate = std::max(1.0, control_rate_hz_);
        const std::chrono::duration<double> period(1.0 / clamped_rate);
        control_timer_ =
            create_wall_timer(period, std::bind(&N6dVelocityController::control_step, this));
    }

    // --- Subscription callbacks --------------------------------------------
    // Cache the latest planned path.
    void handle_path(const nav_msgs::msg::Path::SharedPtr msg) {
        // Cache the incoming path and precompute sample points/cumulative lengths for fast
        // projection inside the control loop.
        if (!msg || msg->poses.empty()) {
            RCLCPP_WARN(get_logger(), "Received empty path; holding position.");
            path_.reset();
            goal_reached_logged_ = false;
            return;
        }

        path_ = *msg;
        preprocess_path();
        last_reacquire_time_ = now();
        goal_reached_logged_ = false;

        RCLCPP_INFO(get_logger(), "New path with %zu poses (L=%.2f m).", path_->poses.size(),
                    total_path_length_);
    }

    // Cache the latest pose sample from localization.
    void handle_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { current_pose_ = *msg; }

    // Cache IMU readings for angular velocity feedback.
    void handle_imu(const sensor_msgs::msg::Imu::SharedPtr msg) { last_imu_ = *msg; }

    // --- Control loop ------------------------------------------------------
    // Core PD loop that computes and publishes body-frame twists.
    void control_step() {
        // Main loop: guard for missing data, estimate velocity, project onto path, compute carrot,
        // run PD (translation in world, rotation in body), clamp, publish twist + debug info.
        if (!ready_for_control()) {
            publish_twist_zero();
            return;
        }

        const rclcpp::Time now_time = now();
        const rclcpp::Time pose_stamp = pose_stamp_or_now(*current_pose_, now_time);
        double pose_dt = 1.0 / std::max(1.0, control_rate_hz_);
        bool pose_sample_new = false;

        // Estimate time delta since last pose sample using pose timestamps.
        if (have_last_pose_stamp_) {
            const double delta = (pose_stamp - last_pose_stamp_).seconds();
            if (delta > 1e-4) {
                pose_dt = delta;
                pose_sample_new = true;
                last_pose_stamp_ = pose_stamp;
            } else if (delta < -1e-4) {
                RCLCPP_WARN(get_logger(),
                            "Pose timestamp moved backwards (%.3f s). Resetting velocity estimate.",
                            delta);
                pose_dt = std::max(1e-3, std::abs(delta));
                pose_sample_new = true;
                last_pose_stamp_ = pose_stamp;
                last_position_.reset();
                have_vel_ = false;
            }
        } else {
            have_last_pose_stamp_ = true;
            pose_sample_new = true;
            last_pose_stamp_ = pose_stamp;
        }

        tf2::Vector3 p_w(current_pose_->pose.position.x, current_pose_->pose.position.y,
                         current_pose_->pose.position.z);

        // Estimate velocity in the world frame via pose finite difference + EMA filter.
        tf2::Vector3 v_est_w = have_vel_ ? vel_world_ : tf2::Vector3(0.0, 0.0, 0.0);
        if (pose_sample_new) {
            if (last_position_) {
                const double inv_dt = pose_dt > 1e-4 ? (1.0 / pose_dt) : 0.0;
                if (inv_dt > 0.0) {
                    tf2::Vector3 v_sample = inv_dt * (p_w - *last_position_);
                    if (have_vel_) {
                        v_sample = vel_alpha_ * v_sample + (1.0 - vel_alpha_) * vel_world_;
                    }
                    v_est_w = v_sample;
                    vel_world_ = v_sample;
                    have_vel_ = true;
                    publish_speed(v_est_w.length(), pose_stamp);
                }
            }
            last_position_ = p_w;
        }

        // Determine how far along the path the robot currently projects so we can select the
        // carrot pose relative to that progress (rather than absolute arc length).
        auto [s_robot, seg_idx, seg_t] = project_onto_path(p_w);
        (void)seg_idx;
        (void)seg_t;

        // Periodically velocity a fresh projection to avoid sticking to stale segments.
        if ((now_time - last_reacquire_time_).seconds() > path_reacquire_period_) {
            // Periodic reacquire handles temporary deviations so projection stays fresh.
            last_reacquire_time_ = now_time;
            std::tie(s_robot, seg_idx, seg_t) = project_onto_path(p_w);
        }

        const double s_remaining = std::max(0.0, total_path_length_ - s_robot);
        double goal_distance = s_remaining;
        if (path_ && !path_->poses.empty()) {
            const auto& goal_pos = path_->poses.back().pose.position;
            const tf2::Vector3 p_goal(goal_pos.x, goal_pos.y, goal_pos.z);
            goal_distance = std::min(goal_distance, (p_w - p_goal).length());
        }
        double adaptive_lookahead = lookahead_distance_;
        double ff_speed = feedforward_speed_;
        // Scale lookahead and feedforward as we approach the goal.
        if (approach_slowdown_distance_ > 1e-3 && goal_distance < approach_slowdown_distance_) {
            const double scale = std::clamp(goal_distance / approach_slowdown_distance_, 0.0, 1.0);
            adaptive_lookahead = std::max(0.02, lookahead_distance_ * scale);
            ff_speed *= scale;
        }
        // Determine the carrot pose by sampling ahead along the path while respecting the total
        // length.
        const double s_target = std::min(s_robot + adaptive_lookahead, total_path_length_);
        const PathSample target_sample = sample_path_pose(s_target);
        geometry_msgs::msg::Pose target_pose = target_sample.pose;
        std::optional<geometry_msgs::msg::Pose> projected_pose;
        if (path_points_.size() >= 2) {
            projected_pose = sample_path_pose(s_robot).pose;
        }
        // Optionally force the carrot orientation to match the final goal pose.
        if (use_goal_orientation_ && path_ && !path_->poses.empty()) {
            target_pose.orientation = path_->poses.back().pose.orientation;
        }

        const std::size_t total_segments = path_points_.size() > 0 ? path_points_.size() - 1 : 0;

        tf2::Vector3 p_des(target_pose.position.x, target_pose.position.y, target_pose.position.z);
        // Desired velocity defaults to zero but may include feedforward along the path tangent.
        tf2::Vector3 v_des(0.0, 0.0, 0.0);
        if (ff_speed > 1e-6) {
            // Use the path tangent to translate feedforward scalar speed into a vector velocity.
            const tf2::Vector3 tangent = path_tangent_at(s_target);
            v_des = tangent * ff_speed;
        }

        tf2::Quaternion q_wb(current_pose_->pose.orientation.x, current_pose_->pose.orientation.y,
                             current_pose_->pose.orientation.z, current_pose_->pose.orientation.w);
        if (q_wb.length2() < 1e-12) {
            q_wb.setValue(0, 0, 0, 1);
        }
        q_wb.normalize();
        tf2::Matrix3x3 R_wb(q_wb);
        const tf2::Matrix3x3 R_bw = R_wb.transpose();

        // --- Linear PD in world, then rotate to body -----------------------
        tf2::Vector3 e_pos_w = p_des - p_w;
        tf2::Vector3 e_vel_w = v_des - v_est_w;
        const tf2::Vector3 e_pos_b = R_bw * e_pos_w;
        const tf2::Vector3 e_vel_b = R_bw * e_vel_w;
        tf2::Vector3 linear_cmd_b(gains_pos_.kp[0] * e_pos_b.x() + gains_pos_.kd[0] * e_vel_b.x(),
                                  gains_pos_.kp[1] * e_pos_b.y() + gains_pos_.kd[1] * e_vel_b.y(),
                                  gains_pos_.kp[2] * e_pos_b.z() + gains_pos_.kd[2] * e_vel_b.z());
        const double speed = v_est_w.length();

        // --- Attitude PD in body frame -------------------------------------
        tf2::Quaternion q_wb_des(target_pose.orientation.x, target_pose.orientation.y,
                                 target_pose.orientation.z, target_pose.orientation.w);
        if (q_wb_des.length2() < 1e-12) {
            q_wb_des = q_wb;
        }
        q_wb_des.normalize();

        // Compute rotation error quaternion (world->body), ensuring shortest path.
        tf2::Quaternion q_err = q_wb.inverse() * q_wb_des;
        if (q_err.w() < 0.0) {
            q_err = tf2::Quaternion(-q_err.x(), -q_err.y(), -q_err.z(), -q_err.w());
        }
        q_err.normalize();
        tf2::Vector3 e_rot(2.0 * q_err.x(), 2.0 * q_err.y(), 2.0 * q_err.z());

        // Angular velocity feedback from IMU (body frame).
        tf2::Vector3 omega_b(0.0, 0.0, 0.0);
        if (last_imu_) {
            omega_b.setX(last_imu_->angular_velocity.x);
            omega_b.setY(last_imu_->angular_velocity.y);
            omega_b.setZ(last_imu_->angular_velocity.z);
        }

        // PD control for angular velocity in body frame.
        tf2::Vector3 angular_cmd_b(gains_att_.kp[0] * e_rot.x() - gains_att_.kd[0] * omega_b.x(),
                                   gains_att_.kp[1] * e_rot.y() - gains_att_.kd[1] * omega_b.y(),
                                   gains_att_.kp[2] * e_rot.z() - gains_att_.kd[2] * omega_b.z());

        // --- Clamp and publish ---------------------------------------------
        linear_cmd_b = clamp_each(linear_cmd_b, max_linear_cmd_body_);
        angular_cmd_b = clamp_each(angular_cmd_b, max_angular_cmd_body_);
        constexpr std::chrono::milliseconds kInfoThrottle{1000};
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kInfoThrottle.count(),
                             "Segment %zu/%zu t=%.2f s=%.2f/%.2f |e_pos|=%.2f m "
                             "|linear_cmd_b|=%.2f |angular_cmd_b|=%.2f speed=%.2f m/s",
                             target_sample.segment_index, total_segments,
                             target_sample.segment_t, s_target, total_path_length_,
                             e_pos_w.length(), linear_cmd_b.length(), angular_cmd_b.length(),
                             speed);
        publish_debug_outputs(target_pose, projected_pose, e_pos_w, e_rot, now_time);

        // If within goal tolerances, hold position instead of issuing new commands.
        if (is_goal_reached(p_w, q_wb)) {
            if (!goal_reached_logged_) {
                RCLCPP_INFO(get_logger(), "Goal reached. Holding position at s=%.2f/%.2f.", s_robot,
                            total_path_length_);
                goal_reached_logged_ = true;
            }
            publish_twist_zero();
            return;
        }
        goal_reached_logged_ = false;
        publish_twist(linear_cmd_b, angular_cmd_b);
    }

    // Check that we have both a path and pose to work with.
    bool ready_for_control() const { return path_ && current_pose_ && !path_->poses.empty(); }

    // Publish a body-frame twist command.
    void publish_twist(const tf2::Vector3& linear_b, const tf2::Vector3& angular_b) {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = linear_b.x();
        twist.linear.y = linear_b.y();
        twist.linear.z = linear_b.z();
        twist.angular.x = angular_b.x();
        twist.angular.y = angular_b.y();
        twist.angular.z = angular_b.z();
        velocity_pub_->publish(twist);
    }

    // Publish a zero twist to hold the vehicle in place.
    void publish_twist_zero() {
        geometry_msgs::msg::Twist twist;
        velocity_pub_->publish(twist);
    }

    // Emit a scalar speed debug message when enabled.
    void publish_speed(double speed, const rclcpp::Time& stamp) {
        if (!debug_enabled_ || !speed_pub_) {
            return;
        }
        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(speed);
        speed_pub_->publish(msg);
    }

    // Publish carrot pose, projection, and error vectors for debugging.
    void publish_debug_outputs(const geometry_msgs::msg::Pose& target_pose,
                               const std::optional<geometry_msgs::msg::Pose>& projected_pose,
                               const tf2::Vector3& pos_error_w, const tf2::Vector3& rot_error_b,
                               const rclcpp::Time& stamp) {
        if (!debug_enabled_) {
            return;
        }
        if (debug_target_pose_pub_ && current_pose_) {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = stamp;
            pose_msg.header.frame_id = current_pose_->header.frame_id;
            pose_msg.pose = target_pose;
            debug_target_pose_pub_->publish(pose_msg);
        }
        if (debug_projected_pose_pub_ && current_pose_ && projected_pose) {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = stamp;
            pose_msg.header.frame_id = current_pose_->header.frame_id;
            pose_msg.pose = *projected_pose;
            debug_projected_pose_pub_->publish(pose_msg);
        }
        if (debug_error_pub_ && current_pose_) {
            geometry_msgs::msg::TwistStamped error_msg;
            error_msg.header.stamp = stamp;
            error_msg.header.frame_id = current_pose_->header.frame_id;
            error_msg.twist.linear.x = pos_error_w.x();
            error_msg.twist.linear.y = pos_error_w.y();
            error_msg.twist.linear.z = pos_error_w.z();
            error_msg.twist.angular.x = rot_error_b.x();
            error_msg.twist.angular.y = rot_error_b.y();
            error_msg.twist.angular.z = rot_error_b.z();
            debug_error_pub_->publish(error_msg);
        }
    }

    // --- Path utilities ----------------------------------------------------
    // Precompute point/length arrays for the current path.
    void preprocess_path() {
        path_points_.clear();
        cumulative_lengths_.clear();
        total_path_length_ = 0.0;

        if (!path_ || path_->poses.size() < 2) {
            return;
        }

        const auto& poses = path_->poses;
        path_points_.reserve(poses.size());
        cumulative_lengths_.reserve(poses.size());

        /* Walk the path to extract points and cumulative arc lengths for fast projection later. */
        for (std::size_t i = 0; i < poses.size(); ++i) {
            const auto& pos = poses[i].pose.position;
            path_points_.emplace_back(pos.x, pos.y, pos.z);

            if (i == 0) {
                cumulative_lengths_.push_back(0.0);
                continue;
            }
            const tf2::Vector3 segment = path_points_[i] - path_points_[i - 1];
            total_path_length_ += segment.length();
            cumulative_lengths_.push_back(total_path_length_);
        }
    }

    // Project a point onto the cached polyline path (arc length, segment index, t).
    std::tuple<double, std::size_t, double> project_onto_path(const tf2::Vector3& p) const {
        if (path_points_.size() < 2) {
            return {0.0, 0, 0.0};
        }

        double best_dist2 = std::numeric_limits<double>::infinity();
        double best_s = 0.0;
        std::size_t best_idx = 0;
        double best_t = 0.0;

        /* Examine each segment and remember the closest projection + arc length. */
        for (std::size_t i = 0; i + 1 < path_points_.size(); ++i) {
            const tf2::Vector3 a = path_points_[i];
            const tf2::Vector3 b = path_points_[i + 1];
            const tf2::Vector3 ab = b - a;
            const double ab2 = ab.length2();

            // Project point p onto segment ab, clamping t to [0,1].
            double t = 0.0;
            if (ab2 > 1e-9) {
                t = std::clamp(((p - a).dot(ab)) / ab2, 0.0, 1.0);
            }

            // Compute squared distance from p to the projection.
            const tf2::Vector3 proj = a + ab * t;
            const double dist2 = (p - proj).length2();
            if (dist2 < best_dist2) {
                best_dist2 = dist2;
                best_idx = i;
                best_t = t;
                const double s0 = (i == 0) ? 0.0 : cumulative_lengths_[i];
                best_s = s0 + (ab.length() * t);
            }
        }

        return {best_s, best_idx, best_t};
    }

    struct PathSample {
        geometry_msgs::msg::Pose pose;
        std::size_t segment_index{0};
        double segment_t{0.0};
    };

    // Sample a pose along the path at a given arc length.
    PathSample sample_path_pose(double s) const {
        PathSample sample;
        if (!path_ || path_points_.size() < 2) {
            return sample;
        }
        s = std::clamp(s, 0.0, total_path_length_);

        std::size_t i = 0;
        // Move to the segment covering the requested arc length.
        while (i + 1 < cumulative_lengths_.size() && cumulative_lengths_[i + 1] < s - 1e-9) {
            ++i;
        }

        // Interpolate linearly along segment i.
        const double s0 = (i == 0) ? 0.0 : cumulative_lengths_[i];
        const double s1 = cumulative_lengths_[std::min(i + 1, cumulative_lengths_.size() - 1)];
        const double denom = std::max(1e-9, s1 - s0);
        const double t = std::clamp((s - s0) / denom, 0.0, 1.0);
        const tf2::Vector3 p = path_points_[i] + (path_points_[i + 1] - path_points_[i]) * t;
        sample.pose.position.x = p.x();
        sample.pose.position.y = p.y();
        sample.pose.position.z = p.z();

        // Slerp orientation along segment i.
        const auto& qa = path_->poses[i].pose.orientation;
        const auto& qb = path_->poses[i + 1].pose.orientation;
        tf2::Quaternion qA(qa.x, qa.y, qa.z, qa.w);
        tf2::Quaternion qB(qb.x, qb.y, qb.z, qb.w);
        if (qA.length2() < 1e-12) {
            qA.setValue(0, 0, 0, 1);
        }
        if (qB.length2() < 1e-12) {
            qB = qA;
        }
        qA.normalize();
        qB.normalize();
        tf2::Quaternion qI = qA.slerp(qB, t);
        qI.normalize();

        sample.pose.orientation.x = qI.x();
        sample.pose.orientation.y = qI.y();
        sample.pose.orientation.z = qI.z();
        sample.pose.orientation.w = qI.w();
        sample.segment_index = i;
        sample.segment_t = t;
        return sample;
    }

    // Compute the tangent direction at arc length s.
    tf2::Vector3 path_tangent_at(double s) const {
        if (path_points_.size() < 2) {
            return tf2::Vector3(0, 0, 0);
        }
        s = std::clamp(s, 0.0, total_path_length_);

        // Move to the segment covering the requested arc length.
        std::size_t i = 0;
        while (i + 1 < cumulative_lengths_.size() && cumulative_lengths_[i + 1] < s - 1e-9) {
            ++i;
        }
        tf2::Vector3 diff = path_points_[i + 1] - path_points_[i];
        const double norm = diff.length();
        if (norm < 1e-9) {
            return tf2::Vector3(0, 0, 0);
        }
        return diff * (1.0 / norm);
    }

    // Check whether position/orientation tolerances relative to the goal are satisfied.
    bool is_goal_reached(const tf2::Vector3& p_w, const tf2::Quaternion& q_wb) const {
        if (!path_ || path_->poses.empty()) {
            return false;
        }

        // Check position tolerance.
        const auto& goal = path_->poses.back().pose;
        const tf2::Vector3 p_goal(goal.position.x, goal.position.y, goal.position.z);
        if ((p_w - p_goal).length() > pos_tolerance_) {
            return false;
        }

        // Check orientation tolerance (yaw only for simplicity).
        tf2::Quaternion q_goal(goal.orientation.x, goal.orientation.y, goal.orientation.z,
                               goal.orientation.w);
        if (q_goal.length2() < 1e-12) {
            return true;
        }
        q_goal.normalize();
        tf2::Quaternion q_err = q_wb.inverse() * q_goal;
        q_err.normalize();
        const double yaw_err = 2.0 * std::atan2(std::sqrt(q_err.z() * q_err.z()), q_err.w());
        return std::abs(yaw_err) < yaw_tolerance_rad_;
    }

    // Helper to turn pose timestamps into rclcpp::Time, falling back to now() if unset.
    rclcpp::Time pose_stamp_or_now(const geometry_msgs::msg::PoseStamped& pose,
                                   const rclcpp::Time& fallback) const {
        rclcpp::Time stamp(pose.header.stamp);
        if (stamp.nanoseconds() == 0) {
            return fallback;
        }
        return stamp;
    }

    // --- Members -----------------------------------------------------------
    // Topics
    std::string path_topic_;
    std::string pose_topic_;
    std::string imu_topic_;
    std::string cmd_velocity_topic_;

    // Parameters
    double control_rate_hz_{50.0};
    double lookahead_distance_{0.8};
    double path_reacquire_period_{0.5};
    double feedforward_speed_{0.0};
    double approach_slowdown_distance_{1.0};
    double vel_alpha_{0.6};
    double pos_tolerance_{0.05};
    double yaw_tolerance_rad_{0.08};
    bool use_goal_orientation_{false};
    bool debug_enabled_{false};
    std::string debug_speed_topic_{"/nav6d/velocity_controller/debug/linear_speed"};
    std::string debug_projected_pose_topic_{"/nav6d/velocity_controller/debug/path_projection"};
    std::string debug_target_pose_topic_{"/nav6d/velocity_controller/debug/carrot_pose"};
    std::string debug_error_topic_{"/nav6d/velocity_controller/debug/control_error"};

    PDGains gains_pos_{};
    PDGains gains_att_{};
    tf2::Vector3 max_linear_cmd_body_{0.6, 0.6, 0.4};
    tf2::Vector3 max_angular_cmd_body_{0.6, 0.6, 0.6};

    // ROS interfaces
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_projected_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_target_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr debug_error_pub_;

    // State
    std::optional<nav_msgs::msg::Path> path_;
    std::optional<geometry_msgs::msg::PoseStamped> current_pose_;
    std::optional<sensor_msgs::msg::Imu> last_imu_;

    std::vector<tf2::Vector3> path_points_;
    std::vector<double> cumulative_lengths_;
    double total_path_length_{0.0};

    tf2::Vector3 vel_world_{0.0, 0.0, 0.0};
    bool have_vel_{false};
    std::optional<tf2::Vector3> last_position_;
    rclcpp::Time last_pose_stamp_{};
    bool have_last_pose_stamp_{false};
    rclcpp::Time last_reacquire_time_{};
    bool goal_reached_logged_{false};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<N6dVelocityController>());
    rclcpp::shutdown();
    return 0;
}
