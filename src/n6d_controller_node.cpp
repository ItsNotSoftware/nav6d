// n6d_controller_node.cpp
//
// ROS 2 node that consumes a nav6d path and applies a 6-DOF PD controller to
// generate body-frame wrench commands for the cmd_bridge.

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
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
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
    auto clamp_axis = [](double v, double limit) { return std::copysign(std::min(std::abs(v), limit), v); };
    return {clamp_axis(value.x(), limits.x()), clamp_axis(value.y(), limits.y()),
            clamp_axis(value.z(), limits.z())};
}
}  // namespace

// ROS 2 node that converts a nav_msgs/Path into wrench commands.
class N6dController : public rclcpp::Node {
   public:
    N6dController() : rclcpp::Node("n6d_controller") {
        declare_parameters();
        init_subscriptions();
        init_publishers();
        init_timer();

        RCLCPP_INFO(get_logger(),
                    "n6d_controller ready. path=%s pose=%s imu=%s -> force=%s (frame=%s)",
                    path_topic_.c_str(), pose_topic_.c_str(), imu_topic_.c_str(), cmd_force_topic_.c_str(),
                    command_frame_.c_str());
    }

   private:
    // --- Parameter loading -------------------------------------------------
    void declare_parameters() {
        // Topic wiring
        path_topic_ = declare_parameter<std::string>("path_topic", "/nav6d/planner/path");
        pose_topic_ = declare_parameter<std::string>("pose_topic", "/space_cobot/pose");
        imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
        cmd_force_topic_ = declare_parameter<std::string>("cmd_force_topic", "/space_cobot/cmd_force");
        command_frame_ = declare_parameter<std::string>("command_frame", "body");

        // Control knobs
        control_rate_hz_ = declare_parameter("control_rate_hz", 50.0);
        lookahead_distance_ = declare_parameter("lookahead_distance", 0.8);
        path_reacquire_period_ = declare_parameter("path_reacquire_period", 0.5);
        feedforward_speed_ = declare_parameter("feedforward_speed", 0.0);
        approach_slowdown_distance_ = declare_parameter("approach_slowdown_distance", 1.0);
        vel_alpha_ = declare_parameter("velocity_ema_alpha", 0.6);
        pos_tolerance_ = declare_parameter("pos_tolerance", 0.05);
        yaw_tolerance_rad_ = declare_parameter("yaw_tolerance_rad", 0.08);
        max_velocity_mps_ = declare_parameter("max_velocity_mps", 0.0);
        velocity_brake_gain_ = declare_parameter("velocity_brake_gain", 6.0);
        debug_enabled_ = declare_parameter("debug_enabled", false);
        debug_speed_topic_ = declare_parameter<std::string>(
            "debug_speed_topic", "/nav6d/controller/debug/linear_speed");
        debug_projected_pose_topic_ = declare_parameter<std::string>(
            "debug_projected_pose_topic", "/nav6d/controller/debug/path_projection");
        debug_target_pose_topic_ = declare_parameter<std::string>(
            "debug_target_pose_topic", "/nav6d/controller/debug/carrot_pose");
        debug_error_topic_ = declare_parameter<std::string>(
            "debug_error_topic", "/nav6d/controller/debug/control_error");

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
        const std::vector<double> max_force =
            declare_parameter<std::vector<double>>("max_force_xyz", {25.0, 25.0, 25.0});
        const std::vector<double> max_torque =
            declare_parameter<std::vector<double>>("max_torque_rpy", {8.0, 8.0, 8.0});

        for (int axis = 0; axis < 3; ++axis) {
            gains_pos_.kp[axis] = axis < static_cast<int>(kp_lin.size()) ? kp_lin[axis] : 0.0;
            gains_pos_.kd[axis] = axis < static_cast<int>(kd_lin.size()) ? kd_lin[axis] : 0.0;
            gains_att_.kp[axis] = axis < static_cast<int>(kp_ang.size()) ? kp_ang[axis] : 0.0;
            gains_att_.kd[axis] = axis < static_cast<int>(kd_ang.size()) ? kd_ang[axis] : 0.0;
        }

        auto pick_limit = [](const std::vector<double>& src, size_t idx) {
            return idx < src.size() ? src[idx] : 0.0;
        };
        max_force_body_ = tf2::Vector3(pick_limit(max_force, 0), pick_limit(max_force, 1),
                                       pick_limit(max_force, 2));
        max_torque_body_ = tf2::Vector3(pick_limit(max_torque, 0), pick_limit(max_torque, 1),
                                        pick_limit(max_torque, 2));
    }

    // --- ROS interface wiring ----------------------------------------------
    void init_subscriptions() {
        path_sub_ =
            create_subscription<nav_msgs::msg::Path>(path_topic_, 10,
                                                     std::bind(&N6dController::handle_path, this,
                                                               std::placeholders::_1));
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, rclcpp::SensorDataQoS(),
            std::bind(&N6dController::handle_pose, this, std::placeholders::_1));
        imu_sub_ =
            create_subscription<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS(),
                                                       std::bind(&N6dController::handle_imu, this,
                                                                 std::placeholders::_1));
    }

    void init_publishers() {
        force_pub_ = create_publisher<geometry_msgs::msg::Wrench>(cmd_force_topic_, 10);
        if (debug_enabled_) {
            if (!debug_speed_topic_.empty()) {
                speed_pub_ =
                    create_publisher<std_msgs::msg::Float32>(debug_speed_topic_, 10);
            }
            if (!debug_projected_pose_topic_.empty()) {
                debug_projected_pose_pub_ =
                    create_publisher<geometry_msgs::msg::PoseStamped>(
                        debug_projected_pose_topic_, 10);
            }
            debug_target_pose_pub_ =
                create_publisher<geometry_msgs::msg::PoseStamped>(debug_target_pose_topic_, 10);
            debug_error_pub_ =
                create_publisher<geometry_msgs::msg::TwistStamped>(debug_error_topic_, 10);
        }
    }

    void init_timer() {
        const double clamped_rate = std::max(1.0, control_rate_hz_);
        const std::chrono::duration<double> period(1.0 / clamped_rate);
        control_timer_ =
            create_wall_timer(period, std::bind(&N6dController::control_step, this));
    }

    // --- Subscription callbacks --------------------------------------------
    void handle_path(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!msg || msg->poses.empty()) {
            RCLCPP_WARN(get_logger(), "Received empty path; holding position.");
            path_.reset();
            return;
        }

        path_ = *msg;
        preprocess_path();
        last_reacquire_time_ = now();

        RCLCPP_INFO(get_logger(), "New path with %zu poses (L=%.2f m).", path_->poses.size(),
                    total_path_length_);
    }

    void handle_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;
    }

    void handle_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
        last_imu_ = *msg;
    }

    // --- Control loop ------------------------------------------------------
    void control_step() {
        if (!ready_for_control()) {
            publish_wrench_zero();
            return;
        }

        const rclcpp::Time now_time = now();
        double dt = 1.0 / std::max(1.0, control_rate_hz_);
        if (have_last_pose_stamp_) {
            dt = std::max(1e-3, (now_time - last_pose_time_).seconds());
        } else {
            have_last_pose_stamp_ = true;
        }
        last_pose_time_ = now_time;

        tf2::Vector3 p_w(current_pose_->pose.position.x, current_pose_->pose.position.y,
                         current_pose_->pose.position.z);

        // Estimate velocity in the world frame via finite difference + EMA filter.
        tf2::Vector3 v_est_w(0.0, 0.0, 0.0);
        if (last_position_) {
            const tf2::Vector3 dp = p_w - *last_position_;
            if (dt > 1e-3) {
                v_est_w = (1.0 / dt) * dp;
            }
            if (have_vel_) {
                v_est_w = vel_alpha_ * v_est_w + (1.0 - vel_alpha_) * vel_world_;
            }
            vel_world_ = v_est_w;
            have_vel_ = true;
            publish_speed(v_est_w.length(), now_time);
        }
        last_position_ = p_w;

        // Determine how far along the path the robot currently projects.
        auto [s_robot, seg_idx, seg_t] = project_onto_path(p_w);
        (void)seg_idx;
        (void)seg_t;

        // Periodically force a fresh projection to avoid sticking to stale segments.
        if ((now_time - last_reacquire_time_).seconds() > path_reacquire_period_) {
            last_reacquire_time_ = now_time;
            std::tie(s_robot, seg_idx, seg_t) = project_onto_path(p_w);
        }

        const double s_remaining = std::max(0.0, total_path_length_ - s_robot);
        double adaptive_lookahead = lookahead_distance_;
        double ff_speed = feedforward_speed_;
        if (approach_slowdown_distance_ > 1e-3 && s_remaining < approach_slowdown_distance_) {
            const double scale = std::clamp(s_remaining / approach_slowdown_distance_, 0.0, 1.0);
            adaptive_lookahead = std::max(0.02, lookahead_distance_ * scale);
            ff_speed *= scale;
        }
        const double s_target = std::min(s_robot + adaptive_lookahead, total_path_length_);
        const PathSample target_sample = sample_path_pose(s_target);
        geometry_msgs::msg::Pose target_pose = target_sample.pose;
        std::optional<geometry_msgs::msg::Pose> projected_pose;
        if (path_points_.size() >= 2) {
            projected_pose = sample_path_pose(s_robot).pose;
        }
        if (path_ && !path_->poses.empty()) {
            target_pose.orientation = path_->poses.back().pose.orientation;
        }

        const std::size_t total_segments = path_points_.size() > 0 ? path_points_.size() - 1 : 0;

        tf2::Vector3 p_des(target_pose.position.x, target_pose.position.y,
                           target_pose.position.z);
        tf2::Vector3 v_des(0.0, 0.0, 0.0);
        if (ff_speed > 1e-6) {
            const tf2::Vector3 tangent = path_tangent_at(s_target);
            v_des = tangent * ff_speed;
        }

        tf2::Quaternion q_wb(current_pose_->pose.orientation.x,
                             current_pose_->pose.orientation.y,
                             current_pose_->pose.orientation.z,
                             current_pose_->pose.orientation.w);
        if (q_wb.length2() < 1e-12) {
            q_wb.setValue(0, 0, 0, 1);
        }
        q_wb.normalize();
        tf2::Matrix3x3 R_wb(q_wb);

        // --- Linear PD in world, then rotate to body -----------------------
        tf2::Vector3 e_pos_w = p_des - p_w;
        tf2::Vector3 e_vel_w = v_des - v_est_w;
        tf2::Vector3 F_w(gains_pos_.kp[0] * e_pos_w.x() + gains_pos_.kd[0] * e_vel_w.x(),
                         gains_pos_.kp[1] * e_pos_w.y() + gains_pos_.kd[1] * e_vel_w.y(),
                         gains_pos_.kp[2] * e_pos_w.z() + gains_pos_.kd[2] * e_vel_w.z());
        const double speed = v_est_w.length();
        if (max_velocity_mps_ > 1e-3) {
            if (speed > max_velocity_mps_) {
                const tf2::Vector3 v_dir = v_est_w * (1.0 / std::max(speed, 1e-6));
                const double excess = speed - max_velocity_mps_;
                F_w += -velocity_brake_gain_ * excess * v_dir;
            }
        }
        tf2::Vector3 F_b = R_wb.transpose() * F_w;

        // --- Attitude PD in body frame -------------------------------------
        tf2::Quaternion q_wb_des(target_pose.orientation.x, target_pose.orientation.y,
                                 target_pose.orientation.z, target_pose.orientation.w);
        if (q_wb_des.length2() < 1e-12) {
            q_wb_des = q_wb;
        }
        q_wb_des.normalize();

        tf2::Quaternion q_err = q_wb.inverse() * q_wb_des;
        if (q_err.w() < 0.0) {
            q_err = tf2::Quaternion(-q_err.x(), -q_err.y(), -q_err.z(), -q_err.w());
        }
        q_err.normalize();
        tf2::Vector3 e_rot(2.0 * q_err.x(), 2.0 * q_err.y(), 2.0 * q_err.z());

        tf2::Vector3 omega_b(0.0, 0.0, 0.0);
        if (last_imu_) {
            omega_b.setX(last_imu_->angular_velocity.x);
            omega_b.setY(last_imu_->angular_velocity.y);
            omega_b.setZ(last_imu_->angular_velocity.z);
        }

        tf2::Vector3 M_b(gains_att_.kp[0] * e_rot.x() - gains_att_.kd[0] * omega_b.x(),
                         gains_att_.kp[1] * e_rot.y() - gains_att_.kd[1] * omega_b.y(),
                         gains_att_.kp[2] * e_rot.z() - gains_att_.kd[2] * omega_b.z());

        F_b = clamp_each(F_b, max_force_body_);
        M_b = clamp_each(M_b, max_torque_body_);

        if (debug_enabled_) {
            RCLCPP_INFO(get_logger(),
                        "Segment %zu/%zu t=%.2f s=%.2f/%.2f |e_pos|=%.2f m |F_b|=%.2f |M_b|=%.2f speed=%.2f m/s",
                        target_sample.segment_index, total_segments, target_sample.segment_t,
                        s_target, total_path_length_, e_pos_w.length(), F_b.length(), M_b.length(),
                        speed);
        } else {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Segment %zu/%zu t=%.2f s=%.2f/%.2f |e_pos|=%.2f m |F_b|=%.2f |M_b|=%.2f speed=%.2f m/s",
                target_sample.segment_index, total_segments, target_sample.segment_t, s_target,
                total_path_length_, e_pos_w.length(), F_b.length(), M_b.length(), speed);
        }

        publish_debug_outputs(target_pose, projected_pose, e_pos_w, e_rot, now_time);

        if (is_goal_reached(p_w, q_wb)) {
            RCLCPP_INFO(get_logger(), "Goal reached. Holding position at s=%.2f/%.2f.", s_robot,
                        total_path_length_);
            publish_wrench_zero();
            return;
        }

        publish_wrench(F_b, M_b);
    }

    bool ready_for_control() const {
        return path_ && current_pose_ && !path_->poses.empty();
    }

    void publish_wrench(const tf2::Vector3& force_b, const tf2::Vector3& torque_b) {
        geometry_msgs::msg::Wrench wrench;
        wrench.force.x = force_b.x();
        wrench.force.y = force_b.y();
        wrench.force.z = force_b.z();
        wrench.torque.x = torque_b.x();
        wrench.torque.y = torque_b.y();
        wrench.torque.z = torque_b.z();
        force_pub_->publish(wrench);
    }

    void publish_wrench_zero() {
        geometry_msgs::msg::Wrench wrench;
        force_pub_->publish(wrench);
    }

    void publish_speed(double speed, const rclcpp::Time& stamp) {
        if (!debug_enabled_ || !speed_pub_) {
            return;
        }
        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(speed);
        speed_pub_->publish(msg);
    }

    void publish_debug_outputs(const geometry_msgs::msg::Pose& target_pose,
                               const std::optional<geometry_msgs::msg::Pose>& projected_pose,
                               const tf2::Vector3& pos_error_w,
                               const tf2::Vector3& rot_error_b, const rclcpp::Time& stamp) {
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

    std::tuple<double, std::size_t, double> project_onto_path(const tf2::Vector3& p) const {
        if (path_points_.size() < 2) {
            return {0.0, 0, 0.0};
        }

        double best_dist2 = std::numeric_limits<double>::infinity();
        double best_s = 0.0;
        std::size_t best_idx = 0;
        double best_t = 0.0;

        for (std::size_t i = 0; i + 1 < path_points_.size(); ++i) {
            const tf2::Vector3 a = path_points_[i];
            const tf2::Vector3 b = path_points_[i + 1];
            const tf2::Vector3 ab = b - a;
            const double ab2 = ab.length2();

            double t = 0.0;
            if (ab2 > 1e-9) {
                t = std::clamp(((p - a).dot(ab)) / ab2, 0.0, 1.0);
            }
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

    PathSample sample_path_pose(double s) const {
        PathSample sample;
        if (!path_ || path_points_.size() < 2) {
            return sample;
        }
        s = std::clamp(s, 0.0, total_path_length_);

        std::size_t i = 0;
        while (i + 1 < cumulative_lengths_.size() && cumulative_lengths_[i + 1] < s - 1e-9) {
            ++i;
        }

        const double s0 = (i == 0) ? 0.0 : cumulative_lengths_[i];
        const double s1 = cumulative_lengths_[std::min(i + 1, cumulative_lengths_.size() - 1)];
        const double denom = std::max(1e-9, s1 - s0);
        const double t = std::clamp((s - s0) / denom, 0.0, 1.0);

        const tf2::Vector3 p = path_points_[i] + (path_points_[i + 1] - path_points_[i]) * t;
        sample.pose.position.x = p.x();
        sample.pose.position.y = p.y();
        sample.pose.position.z = p.z();

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

    tf2::Vector3 path_tangent_at(double s) const {
        if (path_points_.size() < 2) {
            return tf2::Vector3(0, 0, 0);
        }
        s = std::clamp(s, 0.0, total_path_length_);

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

    bool is_goal_reached(const tf2::Vector3& p_w, const tf2::Quaternion& q_wb) const {
        if (!path_ || path_->poses.empty()) {
            return false;
        }

        const auto& goal = path_->poses.back().pose;
        const tf2::Vector3 p_goal(goal.position.x, goal.position.y, goal.position.z);
        if ((p_w - p_goal).length() > pos_tolerance_) {
            return false;
        }

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

    // --- Members -----------------------------------------------------------
    // Topics
    std::string path_topic_;
    std::string pose_topic_;
    std::string imu_topic_;
    std::string cmd_force_topic_;
    std::string command_frame_;

    // Parameters
    double control_rate_hz_{50.0};
    double lookahead_distance_{0.8};
    double path_reacquire_period_{0.5};
    double feedforward_speed_{0.0};
    double approach_slowdown_distance_{1.0};
    double vel_alpha_{0.6};
    double pos_tolerance_{0.05};
    double yaw_tolerance_rad_{0.08};
    double max_velocity_mps_{0.0};
    double velocity_brake_gain_{6.0};
    bool debug_enabled_{false};
    std::string debug_speed_topic_{"/nav6d/controller/debug/linear_speed"};
    std::string debug_projected_pose_topic_{"/nav6d/controller/debug/path_projection"};
    std::string debug_target_pose_topic_{"/nav6d/controller/debug/carrot_pose"};
    std::string debug_error_topic_{"/nav6d/controller/debug/control_error"};

    PDGains gains_pos_{};
    PDGains gains_att_{};
    tf2::Vector3 max_force_body_{25.0, 25.0, 25.0};
    tf2::Vector3 max_torque_body_{8.0, 8.0, 8.0};

    // ROS interfaces
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr force_pub_;
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
    rclcpp::Time last_pose_time_{};
    bool have_last_pose_stamp_{false};
    rclcpp::Time last_reacquire_time_{};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<N6dController>());
    rclcpp::shutdown();
    return 0;
}
