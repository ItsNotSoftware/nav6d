/* n6d_path_evaluator_node.cpp
 *
 * ROS 2 node that evaluates path quality against an OctoMap using geometric and
 * map-based heuristics. It subscribes to nav_msgs/Path plus octomap updates,
 * computes clearance, narrowness, turn angle, and efficiency scores, then
 * publishes a nav6d/PathQuality summary.
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "nav6d/msg/path_quality.hpp"
#include "nav_msgs/msg/path.hpp"
#include "octomap/OcTree.h"
#include "octomap/octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
// Clamp helper for normalized scores.
double clamp_unit(double value) {
    if (value < 0.0) return 0.0;
    if (value > 1.0) return 1.0;
    return value;
}

// Distance helper between two OctoMap points.
double distance_between(const octomap::point3d& a, const octomap::point3d& b) {
    const double dx = static_cast<double>(a.x() - b.x());
    const double dy = static_cast<double>(a.y() - b.y());
    const double dz = static_cast<double>(a.z() - b.z());
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
}  // namespace

// Evaluates nav_msgs/Path quality scores against an OcTree and publishes a summary message.
class N6dPathEvaluator : public rclcpp::Node {
   public:
    // Constructor wires parameters, subscriptions, and publishers.
    N6dPathEvaluator() : rclcpp::Node("n6d_path_evaluator") {
        declare_parameters();
        init_subscriptions();
        init_publishers();

        RCLCPP_INFO(get_logger(),
                    "n6d_path_evaluator ready. map=%s path=%s -> quality=%s",
                    map_topic_.c_str(), path_topic_.c_str(), quality_topic_.c_str());
    }

   private:
    // --- Parameter loading -------------------------------------------------
    // Load topics and evaluation parameters.
    void declare_parameters() {
        map_topic_ = declare_parameter<std::string>("map_topic", "/octomap_full");
        path_topic_ = declare_parameter<std::string>("path_topic", "/nav6d/planner/path");
        quality_topic_ = declare_parameter<std::string>("quality_topic", "/nav6d/path_quality");

        robot_radius_ = declare_parameter("robot_radius", 0.35);
        alpha_ = declare_parameter("alpha", 1.5);
        occupancy_threshold_ = declare_parameter("occupancy_threshold", 0.5);
        sample_step_ = declare_parameter("sample_step", 0.25);

        (void)declare_parameter("w_c", 0.25);
        (void)declare_parameter("w_n", 0.25);
        (void)declare_parameter("w_t", 0.25);
        (void)declare_parameter("w_e", 0.25);
    }

    // --- ROS interface wiring ----------------------------------------------
    // Subscribe to path and map topics.
    void init_subscriptions() {
        map_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
            map_topic_, rclcpp::QoS(1).transient_local(),
            std::bind(&N6dPathEvaluator::handle_map, this, std::placeholders::_1));
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            path_topic_, 10,
            std::bind(&N6dPathEvaluator::handle_path, this, std::placeholders::_1));
    }

    // Create the path quality publisher.
    void init_publishers() {
        quality_pub_ = create_publisher<nav6d::msg::PathQuality>(quality_topic_, 10);
    }

    // --- Subscription callbacks --------------------------------------------
    // Cache the latest OctoMap and refresh the occupied voxel cache.
    void handle_map(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        update_octomap_from_message(msg);
        if (latest_path_) {
            evaluate_and_publish(*latest_path_);
        }
    }

    // Cache the latest path and evaluate when the map is ready.
    void handle_path(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!msg || msg->poses.empty()) {
            RCLCPP_WARN(get_logger(), "Received empty path; skipping evaluation.");
            latest_path_.reset();
            return;
        }
        latest_path_ = *msg;

        if (!octree_) {
            RCLCPP_WARN(get_logger(), "No octomap yet; path quality pending.");
            return;
        }
        evaluate_and_publish(*latest_path_);
    }

    // --- OctoMap handling ---------------------------------------------------
    // Convert the OctoMap message into an OcTree and cache occupied voxel centers.
    void update_octomap_from_message(const octomap_msgs::msg::Octomap::SharedPtr& msg) {
        std::unique_ptr<octomap::AbstractOcTree> abstract(octomap_msgs::msgToMap(*msg));

        if (!abstract) {
            RCLCPP_WARN(get_logger(), "Octomap message could not be converted.");
            return;
        }
        auto* as_oc = dynamic_cast<octomap::OcTree*>(abstract.get());
        if (!as_oc) {
            RCLCPP_WARN(get_logger(), "Octomap is not an OcTree. Unsupported map type.");
            return;
        }
        abstract.release();
        octree_.reset(as_oc);

        rebuild_occupied_cache();
    }

    // Extract occupied voxel centers to accelerate clearance queries.
    void rebuild_occupied_cache() {
        occupied_centers_.clear();
        if (!octree_) return;

        occupied_centers_.reserve(octree_->size());
        for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
            if (it->getOccupancy() >= occupancy_threshold_) {
                occupied_centers_.push_back(it.getCoordinate());
            }
        }

        RCLCPP_INFO(get_logger(), "Cached %zu occupied voxels for clearance queries.",
                    occupied_centers_.size());
    }

    // --- Scoring helpers ----------------------------------------------------
    // Sample path poses directly while honoring the configured sample step.
    std::vector<octomap::point3d> sample_path_points(const nav_msgs::msg::Path& path) const {
        std::vector<octomap::point3d> samples;
        if (path.poses.empty()) return samples;

        samples.reserve(path.poses.size());
        const auto& first = path.poses.front().pose.position;
        octomap::point3d last_sample(first.x, first.y, first.z);
        samples.push_back(last_sample);

        const double step = sample_step_;
        const double min_spacing = 1e-6;
        if (step <= 0.0) {
            for (size_t i = 1; i < path.poses.size(); ++i) {
                const auto& p = path.poses[i].pose.position;
                samples.emplace_back(p.x, p.y, p.z);
            }
            return samples;
        }

        for (size_t i = 1; i < path.poses.size(); ++i) {
            const auto& p = path.poses[i].pose.position;
            const octomap::point3d current(p.x, p.y, p.z);
            const double dist = distance_between(current, last_sample);
            if (dist + min_spacing >= step) {
                samples.push_back(current);
                last_sample = current;
            }
        }

        const auto& last = path.poses.back().pose.position;
        const octomap::point3d last_pose(last.x, last.y, last.z);
        if (distance_between(last_pose, last_sample) > min_spacing) {
            samples.push_back(last_pose);
        }

        return samples;
    }

    // Compute the distance to the nearest occupied voxel center.
    double nearest_occupied_distance(const octomap::point3d& point) const {
        if (occupied_centers_.empty()) {
            return std::numeric_limits<double>::infinity();
        }

        double best_sq = std::numeric_limits<double>::infinity();
        for (const auto& occ : occupied_centers_) {
            const double dx = static_cast<double>(occ.x() - point.x());
            const double dy = static_cast<double>(occ.y() - point.y());
            const double dz = static_cast<double>(occ.z() - point.z());
            const double dist_sq = dx * dx + dy * dy + dz * dz;
            if (dist_sq < best_sq) {
                best_sq = dist_sq;
            }
        }
        return std::sqrt(best_sq);
    }

    // Compute and publish the path quality metrics.
    void evaluate_and_publish(const nav_msgs::msg::Path& path) {
        if (!octree_) return;
        if (path.poses.empty()) return;

        const auto& poses = path.poses;
        const size_t pose_count = poses.size();

        // Path length (L) and straight-line distance (D).
        double path_length = 0.0;
        for (size_t i = 0; i + 1 < pose_count; ++i) {
            const auto& a = poses[i].pose.position;
            const auto& b = poses[i + 1].pose.position;
            path_length += std::hypot(std::hypot(b.x - a.x, b.y - a.y), b.z - a.z);
        }
        const auto& start = poses.front().pose.position;
        const auto& goal = poses.back().pose.position;
        const double straight_dist =
            std::hypot(std::hypot(goal.x - start.x, goal.y - start.y), goal.z - start.z);

        double efficiency_score = 0.0;
        if (path_length > 1e-6) {
            efficiency_score = clamp_unit(straight_dist / path_length);
        } else if (straight_dist <= 1e-6) {
            efficiency_score = 1.0;
        }

        // Turn score based on maximum angle between consecutive segments.
        double max_turn_angle = 0.0;
        bool have_prev = false;
        octomap::point3d prev_dir;
        for (size_t i = 0; i + 1 < pose_count; ++i) {
            const auto& a = poses[i].pose.position;
            const auto& b = poses[i + 1].pose.position;
            const double dx = b.x - a.x;
            const double dy = b.y - a.y;
            const double dz = b.z - a.z;
            const double seg_len = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (seg_len <= 1e-6) {
                continue;
            }
            const octomap::point3d dir(dx / seg_len, dy / seg_len, dz / seg_len);
            if (have_prev) {
                const double dot = std::clamp(static_cast<double>(dir.x() * prev_dir.x() +
                                                                  dir.y() * prev_dir.y() +
                                                                  dir.z() * prev_dir.z()),
                                              -1.0, 1.0);
                const double angle = std::acos(dot);
                if (angle > max_turn_angle) {
                    max_turn_angle = angle;
                }
            }
            prev_dir = dir;
            have_prev = true;
        }
        const double turn_score = 1.0 - clamp_unit(max_turn_angle / M_PI);

        // Clearance and narrowness metrics.
        double clearance_score = 0.0;
        double narrow_score = 0.0;
        if (occupied_centers_.empty()) {
            clearance_score = 1.0;
            narrow_score = 1.0;
        } else {
            const double threshold = std::max(alpha_ * robot_radius_, 1e-6);
            double min_clearance = std::numeric_limits<double>::infinity();
            size_t narrow_count = 0;

            const auto samples = sample_path_points(path);
            for (const auto& sample : samples) {
                const double distance = nearest_occupied_distance(sample);
                const double clearance = distance - robot_radius_;
                min_clearance = std::min(min_clearance, clearance);
                if (clearance < threshold) {
                    ++narrow_count;
                }
            }

            const size_t sample_count = samples.size();
            if (sample_count > 0) {
                clearance_score = clamp_unit(min_clearance / threshold);
                narrow_score = 1.0 - static_cast<double>(narrow_count) / sample_count;
            }
        }

        const double w_c = get_parameter("w_c").as_double();
        const double w_n = get_parameter("w_n").as_double();
        const double w_t = get_parameter("w_t").as_double();
        const double w_e = get_parameter("w_e").as_double();

        nav6d::msg::PathQuality msg;
        msg.clearance_score = static_cast<float>(clearance_score);
        msg.narrow_score = static_cast<float>(narrow_score);
        msg.turn_score = static_cast<float>(turn_score);
        msg.efficiency_score = static_cast<float>(efficiency_score);
        msg.heuristic = static_cast<float>(w_c * clearance_score + w_n * narrow_score +
                                           w_t * turn_score + w_e * efficiency_score);
        quality_pub_->publish(msg);
    }

    // --- Parameters ---------------------------------------------------------
    std::string map_topic_;
    std::string path_topic_;
    std::string quality_topic_;

    double robot_radius_{0.35};
    double alpha_{1.5};
    double occupancy_threshold_{0.5};
    double sample_step_{0.25};

    // --- ROS interfaces -----------------------------------------------------
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<nav6d::msg::PathQuality>::SharedPtr quality_pub_;

    // --- Cached data --------------------------------------------------------
    std::unique_ptr<octomap::OcTree> octree_;
    std::vector<octomap::point3d> occupied_centers_;
    std::optional<nav_msgs::msg::Path> latest_path_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<N6dPathEvaluator>());
    rclcpp::shutdown();
    return 0;
}
