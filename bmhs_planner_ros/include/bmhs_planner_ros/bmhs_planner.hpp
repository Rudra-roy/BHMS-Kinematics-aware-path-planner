#pragma once

#include "bmhs_planner_ros/map_processor.hpp"
#include "bmhs_planner_ros/vehicle_kinematics.hpp"
#include <vector>
#include <optional>

namespace bmhs {

/**
 * @brief Bidirectional Multi-Heuristic Search (BMHS) planner.
 *
 * Simultaneously expands forward and backward search trees using
 * dual 2D Dijkstra heuristics computed in parallel.  Converges when
 * the two trees intersect, then reconstructs and returns the path.
 *
 * Ported from the Python BMHS class.
 */
class BMHSPlanner {
public:
    struct PlanResult {
        std::vector<State> path;         ///< Final (simplified+smoothed) path in pixel coords
        std::vector<State> searched_fwd; ///< Explored nodes (forward tree) — for visualization
        std::vector<State> searched_bwd; ///< Explored nodes (backward tree)
        int iterations = 0;
        double planning_time_s = 0.0;
        bool success = false;
    };

    /**
     * @param map_proc  Loaded and processed map.
     * @param kin       Vehicle kinematics model.
     */
    BMHSPlanner(const MapProcessor& map_proc, const VehicleKinematics& kin);

    /**
     * @brief Plan a path from start to goal (both in pixel coordinates).
     * @param start  Start state (px, py, theta).
     * @param goal   Goal state (px, py, theta).
     * @return PlanResult with the path and diagnostics.
     */
    PlanResult plan(const State& start, const State& goal);

private:
    /// Check if the forward tree node intersects the backward closed set.
    std::optional<GridKey> checkIntersection(
        const GridKey& grid,
        const std::unordered_map<GridKey, State, GridKeyHash>& closed_other) const;

    const MapProcessor&    map_;
    const VehicleKinematics& kin_;

    int max_iterations_ = 500000;
    double weight_ = 1.5;  // WA* heuristic weight
};

}  // namespace bmhs
