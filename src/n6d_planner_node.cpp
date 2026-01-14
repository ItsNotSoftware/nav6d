/* n6d_planner_node.cpp
 *
 * ROS 2 node that generates 3D collision-free paths from OctoMap data. It:
 *   - Loads configurable topics/parameters for integration into various systems.
 *   - Subscribes to map/pose/goal updates, caching the latest messages.
 *   - Runs an A* search (with optional SLERP orientation) when a new goal arrives.
 *   - Publishes nav_msgs/Path messages plus optional RViz markers.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <chrono>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "octomap/OcTree.h"
#include "octomap/octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace {
// Alias used for hashing OcTree keys inside the search containers.
using key_id_t = std::uint64_t;
// Namespace string shared by every marker the planner publishes for RViz.
constexpr char k_marker_namespace[] = "n6d_planner";

// Convert an OcTree key into a dense 64-bit identifier for hashable storage.
key_id_t key_to_id(const octomap::OcTreeKey& key) {
    return (static_cast<key_id_t>(key.k[0]) << 32) | (static_cast<key_id_t>(key.k[1]) << 16) |
           static_cast<key_id_t>(key.k[2]);
}

// Convenience wrapper to get the 3D world coordinate associated with an OcTree key.
octomap::point3d key_to_coord(const octomap::OcTree& tree, const octomap::OcTreeKey& key) {
    return tree.keyToCoord(key);
}

// Euclidean distance helper between two OctoMap points.
double euclidean(const octomap::point3d& a, const octomap::point3d& b) {
    const double dx = static_cast<double>(a.x() - b.x());
    const double dy = static_cast<double>(a.y() - b.y());
    const double dz = static_cast<double>(a.z() - b.z());
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Squared Euclidean distance, used to avoid redundant square roots in comparisons.
double squared_distance(const octomap::point3d& a, const octomap::point3d& b) {
    const double dx = static_cast<double>(a.x() - b.x());
    const double dy = static_cast<double>(a.y() - b.y());
    const double dz = static_cast<double>(a.z() - b.z());
    return dx * dx + dy * dy + dz * dz;
}

// Bookkeeping entry stored in the A* priority queue.
struct QueueEntry {
    key_id_t id{};
    double f_cost{};
};

// Comparator that turns the STL priority queue into a min-heap on `f_cost`.
struct QueueCompare {
    bool operator()(const QueueEntry& a, const QueueEntry& b) const { return a.f_cost > b.f_cost; }
};
}  // namespace

/* ROS 2 node that hosts the nav6d A*-based local planner: constructor wires parameters + ROS
 * interfaces, callbacks cache map/pose/goal messages, and plan_latest_goal() runs A* + publishes
 * the resulting nav_msgs/Path plus markers.
 */
class N6dPlanner : public rclcpp::Node {
   public:
    // Constructor declares parameters, wires ROS interfaces, and announces readiness.
    N6dPlanner() : rclcpp::Node("n6d_planner") {
        // Parameters with defaults
        const std::string map_topic = declare_parameter<std::string>("map_topic", "/octomap_full");
        const std::string pose_topic =
            declare_parameter<std::string>("pose_topic", "/space_cobot/pose");
        const std::string goal_topic = declare_parameter<std::string>("goal_topic", "/nav6d/goal");
        const std::string path_topic =
            declare_parameter<std::string>("path_topic", "/nav6d/planner/path");

        // Planner parameters
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        robot_radius_ = declare_parameter("robot_radius", 0.35);
        occupancy_threshold_ = declare_parameter("occupancy_threshold", 0.5);
        max_search_range_ = declare_parameter("max_search_range", 15.0);
        max_expansions_ = declare_parameter("max_expansions", 60000);
        line_sample_step_ = declare_parameter("line_sample_step", 0.25);
        slerp_orientation_ = declare_parameter("slerp_orientation", false);
        debug_markers_ = declare_parameter("debug_markers", true);
        marker_topic_ =
            declare_parameter<std::string>("marker_topic", "/nav6d/planner/path_markers");

        if (line_sample_step_ <= 0.0) {
            RCLCPP_WARN(
                get_logger(),
                "line_sample_step must be > 0; will set to map resolution when map arrives.");
            line_sample_step_ = 0.0;
        }

        // Subscriptions
        map_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
            map_topic, rclcpp::QoS(1).transient_local(),
            std::bind(&N6dPlanner::map_callback, this, std::placeholders::_1));
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic, rclcpp::SensorDataQoS(),
            std::bind(&N6dPlanner::pose_callback, this, std::placeholders::_1));
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
            std::bind(&N6dPlanner::goal_callback, this, std::placeholders::_1));

        // Publishers
        path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, 10);
        if (debug_markers_) {
            marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
            RCLCPP_INFO(get_logger(), "Debug markers enabled on %s.", marker_topic_.c_str());
        }

        RCLCPP_INFO(
            get_logger(),
            "n6d_planner ready. Only computes on new goal.\n  map:%s pose:%s goal:%s path:%s",
            map_topic.c_str(), pose_topic.c_str(), goal_topic.c_str(), path_topic.c_str());
    }

   private:
    // --- Subscriptions / cached inputs ---
    // map_callback caches OctoMap messages and defers updates while planning.
    // pose_callback tracks the latest robot pose.
    // goal_callback records requested goals and triggers planning when possible.
    // Cache incoming OctoMap updates and trigger re-planning when idle.
    void map_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        if (planning_in_progress_) {
            deferred_octomap_msg_ = msg;  // apply after current plan finishes
            return;
        }
        update_octomap_from_message(msg);
    }

    // Track the robot pose; planning triggers only when both pose and goal exist.
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;  // store latest; does NOT trigger planning
    }

    // Receive user goals and kick off the planner if idle.
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        pending_goal_ = *msg;  // always keep the latest requested goal
        RCLCPP_INFO(get_logger(), "New goal received: (%.2f, %.2f, %.2f).", msg->pose.position.x,
                    msg->pose.position.y, msg->pose.position.z);

        if (planning_in_progress_) {
            RCLCPP_INFO(get_logger(), "Planning in progress; this goal will run next.");
            return;
        }
        plan_latest_goal();  // trigger immediately
    }

    // Validate inputs, run the planner, and publish results (goal-triggered only).
    void plan_latest_goal() {
        if (!pending_goal_) return;

        if (!octree_) {
            RCLCPP_WARN(
                get_logger(),
                "No octomap yet; cannot plan. Keep publishing a goal when map is available.");
            return;
        }
        if (!current_pose_) {
            RCLCPP_WARN(
                get_logger(),
                "No current pose yet; cannot plan. Keep publishing a goal when pose is available.");
            return;
        }

        planning_in_progress_ = true;  // mark busy (no replans until done)
        goal_pose_ = pending_goal_;
        pending_goal_.reset();

        nav_msgs::msg::Path path_msg;

        // --- measure only the planning step ---
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = plan_path(path_msg);  // core planning call
        const auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        RCLCPP_INFO(get_logger(), "Plan computation time: %.3f ms (%s).", ms,
                    ok ? "success" : "failure");
        // --------------------------------------

        // Publish results if successful, otherwise clear any existing markers.
        if (ok) {
            path_pub_->publish(path_msg);
            publish_debug_markers(path_msg);
            RCLCPP_INFO(get_logger(), "Published path with %zu poses (length %.2f m).",
                        path_msg.poses.size(), estimate_path_length(path_msg));
        } else {
            clear_debug_markers();
            RCLCPP_WARN(get_logger(), "Failed to find a collision-free path.");
        }

        planning_in_progress_ = false;  // mark idle

        // Process any deferred map or goal updates now.
        if (deferred_octomap_msg_) {
            update_octomap_from_message(deferred_octomap_msg_);
            deferred_octomap_msg_.reset();
        }

        // Check for a new pending goal to plan next.
        if (pending_goal_) {
            plan_latest_goal();
        }
    }

    // Convert the incoming OctoMap message and cache for collision checks (updates line step).
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
        // Transfer ownership
        abstract.release();
        octree_.reset(as_oc);

        if (line_sample_step_ <= 0.0) {
            line_sample_step_ = octree_->getResolution() * 0.5;
            RCLCPP_INFO(get_logger(), "line_sample_step set to %.3f m.", line_sample_step_);
        }
    }

    // Main planning entry point: straight-line feasibility check followed by A* if needed.
    bool plan_path(nav_msgs::msg::Path& path_out) {
        const auto& start_p = current_pose_->pose.position;
        const auto& goal_p = goal_pose_->pose.position;
        RCLCPP_INFO(get_logger(), "Planning from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f).",
                    start_p.x, start_p.y, start_p.z, goal_p.x, goal_p.y, goal_p.z);

        if (!octree_) return false;

        // Convert start/goal positions into the discrete OcTree representation.
        octomap::OcTreeKey start_key, goal_key;
        if (!octree_->coordToKeyChecked(start_p.x, start_p.y, start_p.z, start_key)) {
            RCLCPP_WARN(get_logger(), "Start is outside map bounds.");
            return false;
        }
        if (!octree_->coordToKeyChecked(goal_p.x, goal_p.y, goal_p.z, goal_key)) {
            RCLCPP_WARN(get_logger(), "Goal is outside map bounds.");
            return false;
        }

        const octomap::point3d start_c = key_to_coord(*octree_, start_key);
        const octomap::point3d goal_c = key_to_coord(*octree_, goal_key);
        const octomap::point3d start_exact(start_p.x, start_p.y, start_p.z);
        const octomap::point3d goal_exact(goal_p.x, goal_p.y, goal_p.z);

        // Drop quests that begin or end inside occupied voxels.
        if (!is_collision_free(start_c)) {
            RCLCPP_WARN(get_logger(), "Start in collision.");
            return false;
        }
        if (!is_collision_free(goal_c)) {
            RCLCPP_WARN(get_logger(), "Goal in collision.");
            return false;
        }

        const geometry_msgs::msg::Quaternion start_q = current_pose_->pose.orientation;
        const geometry_msgs::msg::Quaternion goal_q = goal_pose_->pose.orientation;

        // Handle degenerate case where start and goal are extremely close.
        if (euclidean(start_c, goal_c) < 1e-3) {
            path_out = nav_msgs::msg::Path();
            path_out.header.stamp = now();
            path_out.header.frame_id = map_frame_;
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_out.header;
            pose.pose.position.x = start_exact.x();
            pose.pose.position.y = start_exact.y();
            pose.pose.position.z = start_exact.z();
            pose.pose.orientation =
                slerp_orientation_ ? slerp_between(start_q, goal_q, 1.0) : goal_q;
            path_out.poses.push_back(pose);
            return true;
        }

        // Try straight line first
        if (is_line_free(start_c, goal_c)) {
            path_out =
                create_straight_path(start_c, goal_c, start_exact, goal_exact, start_q, goal_q);
            return true;
        }

        // Fall back to an A* search on the voxel grid when the straight line is blocked.
        std::vector<octomap::OcTreeKey> key_path;
        if (!perform_a_star(start_key, goal_key, start_c, goal_c, key_path)) {
            return false;
        }
        path_out = keys_to_path(key_path, start_exact, goal_exact, start_q, goal_q);
        return true;
    }

    // OctoMap A* expansion using 26-connected voxel moves and a Euclidean heuristic. start_key /
    // goal_key reference the discrete voxels nearest the robot/goal, and result_path stores the
    // ordered OcTree keys that form the final route.
    bool perform_a_star(const octomap::OcTreeKey& start_key, const octomap::OcTreeKey& goal_key,
                        const octomap::point3d& start_coord, const octomap::point3d& goal_coord,
                        std::vector<octomap::OcTreeKey>& result_path) {
        const double res = octree_->getResolution();
        const double max_range2 = (max_search_range_ > 0.0)
                                      ? max_search_range_ * max_search_range_
                                      : std::numeric_limits<double>::infinity();

        // Enumerate the 26 neighbor directions (von Neumann + diagonals) scaled by map resolution.
        std::vector<octomap::point3d> dirs;
        dirs.reserve(26);
        const float step = static_cast<float>(res);
        /* Enumerate all 26-connected neighbor directions (scaled later by resolution). */
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz)
                    if (dx || dy || dz) dirs.emplace_back(dx * step, dy * step, dz * step);

        // A* search containers
        std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> open;
        std::unordered_set<key_id_t> closed;
        std::unordered_map<key_id_t, double> path_costs;
        std::unordered_map<key_id_t, key_id_t> parent;
        std::unordered_map<key_id_t, octomap::OcTreeKey> key_of;

        // Initialize the search with the start node.
        const key_id_t start_id = key_to_id(start_key);
        const key_id_t goal_id = key_to_id(goal_key);
        path_costs[start_id] = 0.0;
        parent[start_id] = start_id;
        key_of[start_id] = start_key;
        open.push({start_id, euclidean(start_coord, goal_coord)});

        // Main A* expansion loop.
        std::size_t expansions = 0;
        /* Expand nodes until the open list is exhausted or the goal is found. */
        while (!open.empty()) {
            // Enforce a hard limit on expansions to prevent pathological cases.
            if (expansions++ > static_cast<std::size_t>(max_expansions_)) {
                RCLCPP_WARN(get_logger(), "A* expansion limit reached (%d).", max_expansions_);
                return false;
            }

            const auto cur = open.top();
            open.pop();
            if (closed.count(cur.id)) continue;

            const octomap::OcTreeKey cur_key = key_of.at(cur.id);
            const octomap::point3d cur_c = key_to_coord(*octree_, cur_key);

            // Goal test
            if (cur.id == goal_id) {
                reconstruct_path(goal_id, start_id, parent, key_of, result_path);
                return true;
            }

            closed.insert(cur.id);

            // Expand neighbors
            /* Visit each neighbor direction for the current voxel. */
            for (const auto& d : dirs) {
                const octomap::point3d nb_c = cur_c + d;

                // Enforce maximum search range from start.
                if (squared_distance(nb_c, start_coord) > max_range2) continue;

                // Convert neighbor coordinate back into an OcTree key.
                octomap::OcTreeKey nb_key;
                if (!octree_->coordToKeyChecked(nb_c.x(), nb_c.y(), nb_c.z(), nb_key)) continue;

                const key_id_t nb_id = key_to_id(nb_key);
                if (closed.count(nb_id)) continue;       // already expanded
                if (!is_collision_free(nb_c)) continue;  // occupied voxel

                // Tentative g cost through current node.
                const double tentative_g = path_costs[cur.id] + euclidean(cur_c, nb_c);
                auto cost_it = path_costs.find(nb_id);
                if (cost_it != path_costs.end() && tentative_g >= cost_it->second) continue;

                // Record best path to this neighbor found so far.
                path_costs[nb_id] = tentative_g;
                parent[nb_id] = cur.id;
                key_of[nb_id] = nb_key;
                const double h = euclidean(nb_c, goal_coord);  // heuristic (Euclidean distance)
                open.push({nb_id, tentative_g + h});           // f = g + h
            }
        }
        return false;
    }

    // Walk the parent map recovered from A* and produce the ordered OcTree key path.
    void reconstruct_path(key_id_t goal_id, key_id_t start_id,
                          const std::unordered_map<key_id_t, key_id_t>& parent,
                          const std::unordered_map<key_id_t, octomap::OcTreeKey>& key_of,
                          std::vector<octomap::OcTreeKey>& out) const {
        out.clear();
        key_id_t cur = goal_id;

        // Walk backwards from goal to start.
        while (true) {
            auto it = key_of.find(cur);
            if (it == key_of.end()) break;
            out.push_back(it->second);
            if (cur == start_id) break;
            auto ip = parent.find(cur);
            if (ip == parent.end()) break;
            cur = ip->second;
        }
        std::reverse(out.begin(), out.end());
    }

    // Transform OcTree keys into a ROS Path message anchored in the map frame.
    nav_msgs::msg::Path keys_to_path(const std::vector<octomap::OcTreeKey>& keys,
                                     const octomap::point3d& start_exact,
                                     const octomap::point3d& goal_exact,
                                     const geometry_msgs::msg::Quaternion& start_q,
                                     const geometry_msgs::msg::Quaternion& goal_q) const {
        nav_msgs::msg::Path path;
        path.header.stamp = now();
        path.header.frame_id = map_frame_;

        if (!keys.empty()) {
            std::vector<octomap::point3d> coords;
            coords.reserve(keys.size());
            /* Convert each discrete key back into a metric coordinate. */
            for (const auto& k : keys) coords.push_back(key_to_coord(*octree_, k));

            path.poses.reserve(coords.size());
            coords.front() = start_exact;
            coords.back() = goal_exact;
            /* Populate PoseStamped entries (with SLERP orientation if enabled). */
            for (std::size_t i = 0; i < coords.size(); ++i) {
                geometry_msgs::msg::PoseStamped p;
                p.header = path.header;
                p.pose.position.x = coords[i].x();
                p.pose.position.y = coords[i].y();
                p.pose.position.z = coords[i].z();
                p.pose.orientation = orientation_for_path(coords, i, start_q, goal_q);
                path.poses.push_back(p);
            }
        }
        return path;
    }

    // Produce a discretised straight-line path along the requested segment.
    nav_msgs::msg::Path create_straight_path(const octomap::point3d& a,
                                             const octomap::point3d& b,
                                             const octomap::point3d& start_exact,
                                             const octomap::point3d& goal_exact,
                                             const geometry_msgs::msg::Quaternion& start_q,
                                             const geometry_msgs::msg::Quaternion& goal_q) const {
        nav_msgs::msg::Path path;
        path.header.stamp = now();
        path.header.frame_id = map_frame_;

        // Convert each key into a PoseStamped waypoint.
        const double dist = euclidean(a, b);
        const double step = std::max(line_sample_step_, 1e-3);
        const int steps = std::max(2, static_cast<int>(std::ceil(dist / step)));
        std::vector<octomap::point3d> coords;
        coords.reserve(static_cast<std::size_t>(steps + 1));
        /* Sample points along the straight segment for collision checking + path output. */
        for (int i = 0; i <= steps; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(steps);
            const auto pnt = a + (b - a) * static_cast<float>(t);
            coords.push_back(pnt);
        }

        // Interpolate waypoints along the segment.
        path.poses.reserve(coords.size());
        coords.front() = start_exact;
        coords.back() = goal_exact;
        /* Convert samples into PoseStamped messages with appropriate orientation. */
        for (std::size_t i = 0; i < coords.size(); ++i) {
            geometry_msgs::msg::PoseStamped p;
            p.header = path.header;
            p.pose.position.x = coords[i].x();
            p.pose.position.y = coords[i].y();
            p.pose.position.z = coords[i].z();
            p.pose.orientation = orientation_for_path(coords, i, start_q, goal_q);
            path.poses.push_back(p);
        }
        return path;
    }

    geometry_msgs::msg::Quaternion orientation_for_path(
        const std::vector<octomap::point3d>& coords, std::size_t idx,
        const geometry_msgs::msg::Quaternion& start_q,
        const geometry_msgs::msg::Quaternion& goal_q) const {
        if (slerp_orientation_) {
            const double denom = coords.size() > 1 ? static_cast<double>(coords.size() - 1) : 1.0;
            const double t = std::clamp(static_cast<double>(idx) / denom, 0.0, 1.0);
            return slerp_between(start_q, goal_q, t);
        }
        return orientation_from_path_index(coords, idx);
    }

    geometry_msgs::msg::Quaternion orientation_from_path_index(
        const std::vector<octomap::point3d>& coords, std::size_t idx) const {
        geometry_msgs::msg::Quaternion q;
        q.w = 1.0;
        if (coords.size() <= 1) return q;

        octomap::point3d tangent;
        if (idx == 0) {
            tangent = coords[1] - coords[0];
        } else if (idx >= coords.size() - 1) {
            tangent = coords[idx] - coords[idx - 1];
        } else {
            tangent = coords[idx + 1] - coords[idx - 1];
        }
        return orientation_from_tangent(tangent);
    }

    geometry_msgs::msg::Quaternion orientation_from_tangent(const octomap::point3d& tangent) const {
        geometry_msgs::msg::Quaternion q;
        tf2::Vector3 x_axis(static_cast<double>(tangent.x()), static_cast<double>(tangent.y()),
                            static_cast<double>(tangent.z()));
        if (x_axis.length2() < 1e-8) {
            q.w = 1.0;
            return q;
        }
        x_axis.normalize();

        tf2::Vector3 z_axis(0.0, 0.0, 1.0);
        if (std::abs(x_axis.dot(z_axis)) > 0.95) z_axis = tf2::Vector3(0.0, 1.0, 0.0);
        tf2::Vector3 y_axis = z_axis.cross(x_axis);
        if (y_axis.length2() < 1e-8) {
            z_axis = tf2::Vector3(1.0, 0.0, 0.0);
            y_axis = z_axis.cross(x_axis);
        }
        y_axis.normalize();
        tf2::Vector3 z_ortho = x_axis.cross(y_axis);
        if (z_ortho.length2() < 1e-8) {
            z_ortho = tf2::Vector3(0.0, 0.0, 1.0);
        } else {
            z_ortho.normalize();
        }

        tf2::Matrix3x3 basis(x_axis.x(), y_axis.x(), z_ortho.x(), x_axis.y(), y_axis.y(),
                             z_ortho.y(), x_axis.z(), y_axis.z(), z_ortho.z());

        tf2::Quaternion quat;
        basis.getRotation(quat);
        quat.normalize();
        q.x = quat.x();
        q.y = quat.y();
        q.z = quat.z();
        q.w = quat.w();
        return q;
    }

    geometry_msgs::msg::Quaternion slerp_between(const geometry_msgs::msg::Quaternion& start,
                                                 const geometry_msgs::msg::Quaternion& goal,
                                                 double t) const {
        geometry_msgs::msg::Quaternion out;
        tf2::Quaternion qs(start.x, start.y, start.z, start.w);
        tf2::Quaternion qg(goal.x, goal.y, goal.z, goal.w);
        if (qs.length2() < 1e-8) qs = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
        if (qg.length2() < 1e-8) qg = qs;
        qs.normalize();
        qg.normalize();
        if (!std::isfinite(t)) t = 0.0;
        t = std::clamp(t, 0.0, 1.0);
        tf2::Quaternion qi = qs.slerp(qg, t);
        if (qi.length2() < 1e-8) qi = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
        else qi.normalize();
        out.x = qi.x();
        out.y = qi.y();
        out.z = qi.z();
        out.w = qi.w();
        return out;
    }

    // --- Collision / line checks ---
    // Brute-force collision checks that dilate occupied voxels with a configurable radius.
    bool is_collision_free(const octomap::point3d& p) const {
        if (!octree_) return false;
        const double res = octree_->getResolution();
        const int r_cells = std::max(1, static_cast<int>(std::ceil(robot_radius_ / res)));

        // Check all voxels within the robot radius.
        /* Iterate over every voxel inside the inflated radius and check occupancy. */
        for (int dx = -r_cells; dx <= r_cells; ++dx)
            for (int dy = -r_cells; dy <= r_cells; ++dy)
                for (int dz = -r_cells; dz <= r_cells; ++dz) {
                    // Skip voxels outside the robot radius.
                    const octomap::point3d s = p + octomap::point3d(dx * static_cast<float>(res),
                                                                    dy * static_cast<float>(res),
                                                                    dz * static_cast<float>(res));
                    if (squared_distance(s, p) > robot_radius_ * robot_radius_) continue;
                    octomap::OcTreeNode* node = octree_->search(s);
                    if (!node) continue;
                    if (node->getOccupancy() >= occupancy_threshold_) return false;
                }
        return true;
    }

    // Sample a segment and ensure every waypoint remains collision free.
    bool is_line_free(const octomap::point3d& a, const octomap::point3d& b) const {
        if (!octree_) return false;
        const double dist = euclidean(a, b);
        const double step =
            std::max(line_sample_step_, static_cast<double>(octree_->getResolution()));
        const int N = std::max(2, static_cast<int>(std::ceil(dist / step)));

        // Sample along the line segment.
        for (int i = 0; i <= N; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(N);
            const auto p = a + (b - a) * static_cast<float>(t);
            if (!is_collision_free(p)) return false;
        }
        return true;
    }

    // Convenience helper for logging the length of the current plan.
    double estimate_path_length(const nav_msgs::msg::Path& path) const {
        if (path.poses.size() < 2) return 0.0;
        double L = 0.0;

        // Sum up straight-line distances between consecutive waypoints.
        for (std::size_t i = 1; i < path.poses.size(); ++i) {
            const auto& a = path.poses[i - 1].pose.position;
            const auto& b = path.poses[i].pose.position;
            const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
            L += std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        return L;
    }

    // --- Debug markers ---
    // Visualise the path as a line strip, waypoint markers, and start/goal spheres.
    void publish_debug_markers(const nav_msgs::msg::Path& path) {
        if (!marker_pub_ || path.poses.empty()) return;

        visualization_msgs::msg::MarkerArray arr;

        visualization_msgs::msg::Marker clear;
        clear.header = path.header;
        clear.ns = k_marker_namespace;
        clear.id = 0;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(clear);

        visualization_msgs::msg::Marker line;
        line.header = path.header;
        line.ns = k_marker_namespace;
        line.id = 1;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.05;
        line.color.a = 1.0F;
        line.color.r = 0.0F;
        line.color.g = 1.0F;
        line.color.b = 0.3F;
        // Add every path waypoint to the line strip.
        for (const auto& ps : path.poses) line.points.emplace_back(ps.pose.position);
        arr.markers.push_back(line);

        const std::size_t waypoint_count = path.poses.size();
        const int waypoint_id_base = 10;
        // Emit small cubes for each waypoint along the path.
        for (std::size_t i = 0; i < waypoint_count; ++i) {
            visualization_msgs::msg::Marker box;
            box.header = path.header;
            box.ns = k_marker_namespace;
            box.id = waypoint_id_base + static_cast<int>(i);
            box.type = visualization_msgs::msg::Marker::CUBE;
            box.action = visualization_msgs::msg::Marker::ADD;
            box.pose = path.poses[i].pose;
            box.scale.x = 0.22;
            box.scale.y = 0.14;
            box.scale.z = 0.05;
            box.color.a = 0.75F;
            box.color.r = 0.2F;
            box.color.g = 0.8F;
            box.color.b = 1.0F;
            arr.markers.push_back(box);
        }

        const auto& sp = path.poses.front().pose.position;
        const auto& gp = path.poses.back().pose.position;

        visualization_msgs::msg::Marker start;
        start.header = path.header;
        start.ns = k_marker_namespace;
        start.id = 3;
        start.type = visualization_msgs::msg::Marker::CUBE;
        start.action = visualization_msgs::msg::Marker::ADD;
        start.pose.position = sp;
        start.pose.orientation = path.poses.front().pose.orientation;
        start.scale.x = 0.28;
        start.scale.y = 0.20;
        start.scale.z = 0.08;
        start.color.a = 0.9F;
        start.color.r = 0.0F;
        start.color.g = 0.4F;
        start.color.b = 1.0F;
        arr.markers.push_back(start);

        visualization_msgs::msg::Marker goal;
        goal.header = path.header;
        goal.ns = k_marker_namespace;
        goal.id = 4;
        goal.type = visualization_msgs::msg::Marker::CUBE;
        goal.action = visualization_msgs::msg::Marker::ADD;
        goal.pose.position = gp;
        goal.pose.orientation = path.poses.back().pose.orientation;
        goal.scale.x = 0.30;
        goal.scale.y = 0.22;
        goal.scale.z = 0.10;
        goal.color.a = 0.9F;
        goal.color.r = 1.0F;
        goal.color.g = 0.2F;
        goal.color.b = 0.1F;
        arr.markers.push_back(goal);

        marker_pub_->publish(arr);
    }

    // Remove all previously published markers from RViz.
    void clear_debug_markers() {
        if (!marker_pub_) return;
        visualization_msgs::msg::MarkerArray arr;
        visualization_msgs::msg::Marker clear;
        clear.header.frame_id = map_frame_;
        clear.header.stamp = now();
        clear.ns = k_marker_namespace;
        clear.id = 0;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(clear);
        marker_pub_->publish(arr);
    }

    // --- Members: subscriptions/publishers plus cached state used across planning cycles.
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
    // Latest robot pose sample.
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    // Incoming goal requests.
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    // Publisher for the planned path.
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    // Optional publisher for RViz visualisation.
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // Backing occupancy map used for collision checks.
    std::shared_ptr<octomap::OcTree> octree_;
    // Latest pose received from localisation.
    std::optional<geometry_msgs::msg::PoseStamped> current_pose_;
    // Currently executing goal.
    std::optional<geometry_msgs::msg::PoseStamped> goal_pose_;
    // Most recent goal waiting to be processed.
    std::optional<geometry_msgs::msg::PoseStamped> pending_goal_;

    // params
    double robot_radius_{0.35};        // Collision model radius (m).
    double occupancy_threshold_{0.5};  // Occupancy probability treated as an obstacle.
    double max_search_range_{15.0};    // Maximum allowed straight-line distance (m).
    int max_expansions_{60000};        // Safety limit on A* expansions.
    double line_sample_step_{0.25};    // Step size (m) for straight-line feasibility tests.
    std::string map_frame_{"map"};     // Output frame id.
    bool slerp_orientation_{false};    // Interpolate orientation with SLERP when true.
    bool debug_markers_{false};        // Whether to publish RViz markers.
    std::string marker_topic_;         // MarkerArray output topic.

    // state
    bool planning_in_progress_{false};  // True while the planner is solving the current goal.
    octomap_msgs::msg::Octomap::SharedPtr deferred_octomap_msg_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<N6dPlanner>());
    rclcpp::shutdown();
    return 0;
}
