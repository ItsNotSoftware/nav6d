/* n6d_path_evaluator_node.cpp
 *
 * ROS 2 node that evaluates path quality against an OctoMap using geometric and
 * map-based metrics plus a Monte Carlo robustness estimate. It subscribes to
 * nav_msgs/Path plus octomap updates, computes clearance, narrowness, turn
 * angle, and efficiency scores, then publishes a nav6d/PathQuality summary.
 *
 * The PathQuality.heuristic field is used as the Monte Carlo path goodness
 * score (1 - path_risk) for downstream compatibility.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "nav6d/msg/path_quality.hpp"
#include "nav_msgs/msg/path.hpp"
#include "octomap/OcTree.h"
#include "octomap/octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
    // Clamp helper for normalized scores.
    double clamp_unit(double value)
    {
        if (value < 0.0)
            return 0.0;
        if (value > 1.0)
            return 1.0;
        return value;
    }

    // Distance helper between two OctoMap points.
    double distance_between(const octomap::point3d &a, const octomap::point3d &b)
    {
        const double dx = static_cast<double>(a.x() - b.x());
        const double dy = static_cast<double>(a.y() - b.y());
        const double dz = static_cast<double>(a.z() - b.z());
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
} // namespace

// Evaluates nav_msgs/Path quality scores against an OcTree and publishes a summary message.
class N6dPathEvaluator : public rclcpp::Node
{
public:
    // Constructor wires parameters, subscriptions, and publishers.
    N6dPathEvaluator() : rclcpp::Node("n6d_path_evaluator")
    {
        declare_parameters();
        runtime_log_params_cb_handle_ = add_on_set_parameters_callback(
            std::bind(&N6dPathEvaluator::on_set_parameters, this, std::placeholders::_1));
        init_subscriptions();
        init_publishers();

        RCLCPP_INFO(get_logger(),
                    "n6d_path_evaluator ready. map=%s path=%s -> quality=%s",
                    map_topic_.c_str(), path_topic_.c_str(), quality_topic_.c_str());
    }

private:
    // --- Parameter loading -------------------------------------------------
    // Load topics and evaluation parameters.
    void declare_parameters()
    {
        map_topic_ = declare_parameter<std::string>("map_topic", "/octomap_full");
        path_topic_ = declare_parameter<std::string>("path_topic", "/nav6d/planner/path");
        quality_topic_ = declare_parameter<std::string>("quality_topic", "/nav6d/path_quality");

        robot_radius_ = declare_parameter("robot_radius", 0.35);
        alpha_ = declare_parameter("alpha", 1.5);
        occupancy_threshold_ = declare_parameter("occupancy_threshold", 0.5);
        sample_step_ = declare_parameter("sample_step", 0.25);
        min_path_length_ = declare_parameter("min_path_length", 0.05);
        runtime_info_logs_enabled_ = declare_parameter("runtime_info_logs_enabled", true);

        // Monte Carlo robustness estimator (used for msg.heuristic output).
        mc_particles_per_pose_ = std::max<int64_t>(1, declare_parameter("mc_particles_per_pose", 200));
        mc_sigma_xy_ = std::max(0.0, declare_parameter("mc_sigma_xy", 0.08));
        mc_sigma_z_ = std::max(0.0, declare_parameter("mc_sigma_z", 0.05));
        mc_sigma_clip_ = std::max(0.0, declare_parameter("mc_sigma_clip", 3.0));
        mc_soft_margin_ = std::max(0.0, declare_parameter("mc_soft_margin", 0.10));
        mc_topk_fraction_ =
            std::clamp(declare_parameter("mc_topk_fraction", 0.10), 0.0, 1.0);
    }

    // --- ROS interface wiring ----------------------------------------------
    rcl_interfaces::msg::SetParametersResult on_set_parameters(
        const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &p : params)
        {
            if (p.get_name() == "runtime_info_logs_enabled")
            {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
                {
                    result.successful = false;
                    result.reason = "runtime_info_logs_enabled must be bool";
                    return result;
                }
                runtime_info_logs_enabled_ = p.as_bool();
            }
        }
        return result;
    }

    // --- ROS interface wiring ----------------------------------------------
    // Subscribe to path and map topics.
    void init_subscriptions()
    {
        map_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
            map_topic_, rclcpp::QoS(1).transient_local(),
            std::bind(&N6dPathEvaluator::handle_map, this, std::placeholders::_1));
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            path_topic_, 10,
            std::bind(&N6dPathEvaluator::handle_path, this, std::placeholders::_1));
    }

    // Create the path quality publisher.
    void init_publishers()
    {
        quality_pub_ = create_publisher<nav6d::msg::PathQuality>(quality_topic_, 10);
    }

    // --- Subscription callbacks --------------------------------------------
    // Cache the latest OctoMap and refresh the occupied voxel cache.
    void handle_map(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        update_octomap_from_message(msg);
        if (latest_path_)
        {
            evaluate_and_publish(*latest_path_);
        }
    }

    // Cache the latest path and evaluate when the map is ready.
    void handle_path(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!msg || msg->poses.empty())
        {
            RCLCPP_WARN(get_logger(), "Received empty path; skipping evaluation.");
            latest_path_.reset();
            return;
        }
        latest_path_ = *msg;

        if (!octree_)
        {
            RCLCPP_WARN(get_logger(), "No octomap yet; path quality pending.");
            return;
        }
        evaluate_and_publish(*latest_path_);
    }

    // --- OctoMap handling ---------------------------------------------------
    // Convert the OctoMap message into an OcTree and cache occupied voxel centers.
    void update_octomap_from_message(const octomap_msgs::msg::Octomap::SharedPtr &msg)
    {
        std::unique_ptr<octomap::AbstractOcTree> abstract(octomap_msgs::msgToMap(*msg));

        if (!abstract)
        {
            RCLCPP_WARN(get_logger(), "Octomap message could not be converted.");
            return;
        }
        auto *as_oc = dynamic_cast<octomap::OcTree *>(abstract.get());
        if (!as_oc)
        {
            RCLCPP_WARN(get_logger(), "Octomap is not an OcTree. Unsupported map type.");
            return;
        }
        abstract.release();
        octree_.reset(as_oc);

        rebuild_occupied_cache();
    }

    // Extract occupied voxel centers to accelerate clearance queries.
    void rebuild_occupied_cache()
    {
        occupied_centers_.clear();
        if (!octree_)
            return;

        occupied_centers_.reserve(octree_->size());
        for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it)
        {
            if (it->getOccupancy() >= occupancy_threshold_)
            {
                occupied_centers_.push_back(it.getCoordinate());
            }
        }
    }

    // --- Scoring helpers ----------------------------------------------------
    struct MonteCarloSummary
    {
        double path_risk{0.0};      // aggregate risk in [0,1]
        double mean_local_risk{0.0};
        double max_local_risk{0.0};
        size_t sample_count{0};
    };

    // Sample path poses directly while honoring the configured sample step.
    std::vector<octomap::point3d> sample_path_points(const nav_msgs::msg::Path &path) const
    {
        std::vector<octomap::point3d> samples;
        if (path.poses.empty())
            return samples;

        samples.reserve(path.poses.size());
        const auto &first = path.poses.front().pose.position;
        octomap::point3d last_sample(first.x, first.y, first.z);
        samples.push_back(last_sample);

        const double step = sample_step_;
        const double min_spacing = 1e-6;
        if (step <= 0.0)
        {
            for (size_t i = 1; i < path.poses.size(); ++i)
            {
                const auto &p = path.poses[i].pose.position;
                samples.emplace_back(p.x, p.y, p.z);
            }
            return samples;
        }

        for (size_t i = 1; i < path.poses.size(); ++i)
        {
            const auto &p = path.poses[i].pose.position;
            const octomap::point3d current(p.x, p.y, p.z);
            const double dist = distance_between(current, last_sample);
            if (dist + min_spacing >= step)
            {
                samples.push_back(current);
                last_sample = current;
            }
        }

        const auto &last = path.poses.back().pose.position;
        const octomap::point3d last_pose(last.x, last.y, last.z);
        if (distance_between(last_pose, last_sample) > min_spacing)
        {
            samples.push_back(last_pose);
        }

        return samples;
    }

    // Compute the distance to the nearest occupied voxel center.
    double nearest_occupied_distance(const octomap::point3d &point) const
    {
        if (occupied_centers_.empty())
        {
            return std::numeric_limits<double>::infinity();
        }

        double best_sq = std::numeric_limits<double>::infinity();
        for (const auto &occ : occupied_centers_)
        {
            const double dx = static_cast<double>(occ.x() - point.x());
            const double dy = static_cast<double>(occ.y() - point.y());
            const double dz = static_cast<double>(occ.z() - point.z());
            const double dist_sq = dx * dx + dy * dy + dz * dz;
            if (dist_sq < best_sq)
            {
                best_sq = dist_sq;
            }
        }
        return std::sqrt(best_sq);
    }

    double nearest_occupied_distance_from_candidates(
        const octomap::point3d &point,
        const std::vector<octomap::point3d> &candidates) const
    {
        if (candidates.empty())
        {
            return std::numeric_limits<double>::infinity();
        }

        double best_sq = std::numeric_limits<double>::infinity();
        for (const auto &occ : candidates)
        {
            const double dx = static_cast<double>(occ.x() - point.x());
            const double dy = static_cast<double>(occ.y() - point.y());
            const double dz = static_cast<double>(occ.z() - point.z());
            const double dist_sq = dx * dx + dy * dy + dz * dz;
            if (dist_sq < best_sq)
            {
                best_sq = dist_sq;
            }
        }
        return std::sqrt(best_sq);
    }

    std::vector<octomap::point3d> collect_local_occupied_candidates(
        const octomap::point3d &center,
        double radius) const
    {
        std::vector<octomap::point3d> candidates;
        if (occupied_centers_.empty() || radius <= 0.0)
        {
            return candidates;
        }

        const double r2 = radius * radius;
        for (const auto &occ : occupied_centers_)
        {
            const double dx = static_cast<double>(occ.x() - center.x());
            const double dy = static_cast<double>(occ.y() - center.y());
            const double dz = static_cast<double>(occ.z() - center.z());
            const double dist_sq = dx * dx + dy * dy + dz * dz;
            if (dist_sq <= r2)
            {
                candidates.push_back(occ);
            }
        }
        return candidates;
    }

    // Convert clearance (distance from robot surface to nearest occupied center)
    // into a soft risk contribution in [0,1].
    double soft_collision_risk_from_clearance(double clearance) const
    {
        if (!std::isfinite(clearance))
        {
            return 0.0;
        }
        if (clearance <= 0.0)
        {
            return 1.0;
        }
        if (mc_soft_margin_ <= 1e-9)
        {
            return 0.0;
        }
        return clamp_unit(1.0 - (clearance / mc_soft_margin_));
    }

    MonteCarloSummary estimate_monte_carlo_path_risk(
        const std::vector<octomap::point3d> &samples)
    {
        MonteCarloSummary out;
        out.sample_count = samples.size();
        if (samples.empty() || mc_particles_per_pose_ <= 0 || !octree_)
        {
            return out;
        }
        if (occupied_centers_.empty())
        {
            return out;
        }

        const double sigma_max = std::max(mc_sigma_xy_, mc_sigma_z_);
        const double voxel_pad =
            octree_ ? (0.5 * std::sqrt(3.0) * octree_->getResolution()) : 0.0;
        const double local_search_radius =
            robot_radius_ + mc_soft_margin_ + mc_sigma_clip_ * sigma_max + voxel_pad;

        std::vector<double> local_risks;
        local_risks.reserve(samples.size());

        for (const auto &sample : samples)
        {
            const auto local_candidates =
                collect_local_occupied_candidates(sample, local_search_radius);
            if (local_candidates.empty())
            {
                local_risks.push_back(0.0);
                continue;
            }

            double risk_sum = 0.0;
            for (int64_t i = 0; i < mc_particles_per_pose_; ++i)
            {
                double dx = mc_sigma_xy_ * std_normal_(rng_);
                double dy = mc_sigma_xy_ * std_normal_(rng_);
                double dz = mc_sigma_z_ * std_normal_(rng_);

                if (mc_sigma_clip_ > 0.0)
                {
                    const double clip_xy = mc_sigma_clip_ * mc_sigma_xy_;
                    const double clip_z = mc_sigma_clip_ * mc_sigma_z_;
                    dx = std::clamp(dx, -clip_xy, clip_xy);
                    dy = std::clamp(dy, -clip_xy, clip_xy);
                    dz = std::clamp(dz, -clip_z, clip_z);
                }

                const octomap::point3d perturbed(sample.x() + static_cast<float>(dx),
                                                 sample.y() + static_cast<float>(dy),
                                                 sample.z() + static_cast<float>(dz));

                const double distance =
                    nearest_occupied_distance_from_candidates(perturbed, local_candidates);
                const double clearance = distance - robot_radius_;
                risk_sum += soft_collision_risk_from_clearance(clearance);
            }

            const double local_risk =
                risk_sum / static_cast<double>(std::max<int64_t>(1, mc_particles_per_pose_));
            local_risks.push_back(clamp_unit(local_risk));
        }

        if (local_risks.empty())
        {
            return out;
        }

        out.mean_local_risk =
            std::accumulate(local_risks.begin(), local_risks.end(), 0.0) /
            static_cast<double>(local_risks.size());
        out.max_local_risk = *std::max_element(local_risks.begin(), local_risks.end());

        const double topk_fraction =
            (mc_topk_fraction_ <= 0.0) ? 1.0 / static_cast<double>(local_risks.size())
                                       : mc_topk_fraction_;
        const size_t k = std::min<size_t>(
            local_risks.size(),
            std::max<size_t>(1, static_cast<size_t>(std::ceil(topk_fraction * local_risks.size()))));

        std::nth_element(local_risks.begin(), local_risks.end() - k, local_risks.end());
        const double topk_sum =
            std::accumulate(local_risks.end() - k, local_risks.end(), 0.0);
        out.path_risk = clamp_unit(topk_sum / static_cast<double>(k));
        return out;
    }

    // Compute and publish the path quality metrics.
    void evaluate_and_publish(const nav_msgs::msg::Path &path)
    {
        const auto t_total_0 = std::chrono::steady_clock::now();
        if (!octree_)
            return;
        if (path.poses.empty())
            return;

        const auto &poses = path.poses;
        const size_t pose_count = poses.size();

        // Path length (L) and straight-line distance (D).
        double path_length = 0.0;
        for (size_t i = 0; i + 1 < pose_count; ++i)
        {
            const auto &a = poses[i].pose.position;
            const auto &b = poses[i + 1].pose.position;
            path_length += std::hypot(std::hypot(b.x - a.x, b.y - a.y), b.z - a.z);
        }
        if (path_length < min_path_length_)
        {
            return;
        }
        const auto &start = poses.front().pose.position;
        const auto &goal = poses.back().pose.position;
        const double straight_dist =
            std::hypot(std::hypot(goal.x - start.x, goal.y - start.y), goal.z - start.z);

        double efficiency_score = 0.0;
        if (path_length > 1e-6)
        {
            efficiency_score = clamp_unit(straight_dist / path_length);
        }
        else if (straight_dist <= 1e-6)
        {
            efficiency_score = 1.0;
        }

        // Turn score based on maximum angle between consecutive segments.
        double max_turn_angle = 0.0;
        bool have_prev = false;
        octomap::point3d prev_dir;
        for (size_t i = 0; i + 1 < pose_count; ++i)
        {
            const auto &a = poses[i].pose.position;
            const auto &b = poses[i + 1].pose.position;
            const double dx = b.x - a.x;
            const double dy = b.y - a.y;
            const double dz = b.z - a.z;
            const double seg_len = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (seg_len <= 1e-6)
            {
                continue;
            }
            const octomap::point3d dir(dx / seg_len, dy / seg_len, dz / seg_len);
            if (have_prev)
            {
                const double dot = std::clamp(static_cast<double>(dir.x() * prev_dir.x() +
                                                                  dir.y() * prev_dir.y() +
                                                                  dir.z() * prev_dir.z()),
                                              -1.0, 1.0);
                const double angle = std::acos(dot);
                if (angle > max_turn_angle)
                {
                    max_turn_angle = angle;
                }
            }
            prev_dir = dir;
            have_prev = true;
        }
        const double turn_score = 1.0 - clamp_unit(max_turn_angle / M_PI);

        // Clearance and narrowness metrics.
        const double threshold = std::max(alpha_ * robot_radius_, 1e-6);
        double clearance_score = 0.0;
        double narrow_score = 0.0;
        double min_clearance_m = threshold;
        double narrow_fraction = 0.0;
        const auto t_sample_0 = std::chrono::steady_clock::now();
        const auto samples = sample_path_points(path);
        const auto t_sample_1 = std::chrono::steady_clock::now();
        if (occupied_centers_.empty())
        {
            clearance_score = 1.0;
            narrow_score = 1.0;
        }
        else
        {
            double min_clearance = std::numeric_limits<double>::infinity();
            size_t narrow_count = 0;

            for (const auto &sample : samples)
            {
                const double distance = nearest_occupied_distance(sample);
                const double clearance = distance - robot_radius_;
                min_clearance = std::min(min_clearance, clearance);
                if (clearance < threshold)
                {
                    ++narrow_count;
                }
            }

            const size_t sample_count = samples.size();
            if (sample_count > 0)
            {
                min_clearance_m = min_clearance;
                narrow_fraction = static_cast<double>(narrow_count) / static_cast<double>(sample_count);
                clearance_score = clamp_unit(min_clearance / threshold);
                narrow_score = 1.0 - narrow_fraction;
            }
        }

        const double detour_ratio =
            (straight_dist > 1e-6) ? (path_length / straight_dist) : 1.0;

        const auto t_mc_0 = std::chrono::steady_clock::now();
        const MonteCarloSummary mc = estimate_monte_carlo_path_risk(samples);
        const auto t_mc_1 = std::chrono::steady_clock::now();
        const double mc_goodness = 1.0 - mc.path_risk;

        nav6d::msg::PathQuality msg;
        msg.clearance_score = static_cast<float>(clearance_score);
        msg.narrow_score = static_cast<float>(narrow_score);
        msg.turn_score = static_cast<float>(turn_score);
        msg.efficiency_score = static_cast<float>(efficiency_score);
        // Backward-compatible field now carries Monte Carlo path goodness.
        msg.heuristic = static_cast<float>(clamp_unit(mc_goodness));
        msg.min_clearance_m = static_cast<float>(min_clearance_m);
        msg.narrow_fraction = static_cast<float>(narrow_fraction);
        msg.max_turn_rad = static_cast<float>(max_turn_angle);
        msg.path_length_m = static_cast<float>(path_length);
        msg.straight_dist_m = static_cast<float>(straight_dist);
        msg.detour_ratio = static_cast<float>(detour_ratio);
        quality_pub_->publish(msg);
        const auto t_total_1 = std::chrono::steady_clock::now();

        const double sample_ms =
            std::chrono::duration<double, std::milli>(t_sample_1 - t_sample_0).count();
        const double mc_ms =
            std::chrono::duration<double, std::milli>(t_mc_1 - t_mc_0).count();
        const double total_ms =
            std::chrono::duration<double, std::milli>(t_total_1 - t_total_0).count();

        RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "PathQuality: mc_goodness=%.3f (risk=%.3f mean=%.3f max=%.3f samples=%zu) | "
            "timing total=%.2fms sample=%.2fms mc=%.2fms particles=%ld total_particles=%zu",
            mc_goodness, mc.path_risk, mc.mean_local_risk, mc.max_local_risk, mc.sample_count,
            total_ms, sample_ms, mc_ms, mc_particles_per_pose_,
            samples.size() * static_cast<size_t>(std::max<int64_t>(0, mc_particles_per_pose_)));

        if (runtime_info_logs_enabled_)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "MC timing: total=%.2fms sample=%.2fms mc=%.2fms (samples=%zu particles=%ld total=%zu)",
                total_ms, sample_ms, mc_ms, samples.size(), mc_particles_per_pose_,
                samples.size() * static_cast<size_t>(std::max<int64_t>(0, mc_particles_per_pose_)));
        }
    }

    // --- Parameters ---------------------------------------------------------
    std::string map_topic_;
    std::string path_topic_;
    std::string quality_topic_;

    double robot_radius_{0.35};
    double alpha_{1.5};
    double occupancy_threshold_{0.5};
    double sample_step_{0.25};
    double min_path_length_{0.05};
    bool runtime_info_logs_enabled_{true};
    int64_t mc_particles_per_pose_{200};
    double mc_sigma_xy_{0.08};
    double mc_sigma_z_{0.05};
    double mc_sigma_clip_{3.0};
    double mc_soft_margin_{0.10};
    double mc_topk_fraction_{0.10};
    // --- ROS interfaces -----------------------------------------------------
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<nav6d::msg::PathQuality>::SharedPtr quality_pub_;

    // --- Cached data --------------------------------------------------------
    std::unique_ptr<octomap::OcTree> octree_;
    std::vector<octomap::point3d> occupied_centers_;
    std::optional<nav_msgs::msg::Path> latest_path_;
    std::mt19937 rng_{12345u};
    std::normal_distribution<double> std_normal_{0.0, 1.0};
    OnSetParametersCallbackHandle::SharedPtr runtime_log_params_cb_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<N6dPathEvaluator>());
    rclcpp::shutdown();
    return 0;
}
