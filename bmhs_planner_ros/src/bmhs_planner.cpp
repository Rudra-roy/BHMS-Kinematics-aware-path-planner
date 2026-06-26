#include "bmhs_planner_ros/bmhs_planner.hpp"
#include "bmhs_planner_ros/dijkstra.hpp"
#include "bmhs_planner_ros/path_smoother.hpp"

#include <queue>
#include <unordered_map>
#include <chrono>
#include <limits>
#include <cmath>
#include <iostream>

namespace bmhs {

// ============================================================================
// Priority queue entry for the kinematic A* search
// ============================================================================

struct OpenEntry {
    double f;       // f = g + w*h
    double g;       // g-cost so far
    State  state;   // continuous state

    bool operator>(const OpenEntry& o) const { return f > o.f; }
};

// ============================================================================

BMHSPlanner::BMHSPlanner(const MapProcessor& map_proc, const VehicleKinematics& kin)
    : map_(map_proc)
    , kin_(kin)
{
}

std::optional<GridKey> BMHSPlanner::checkIntersection(
    const GridKey& grid,
    const std::unordered_map<GridKey, State, GridKeyHash>& closed_other) const
{
    int theta_bins = kin_.thetaBins();

    // For differential, orientation doesn't matter (robot can turn in place)
    bool diff = (kin_.type() == VehicleKinematics::Type::DIFFERENTIAL);

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (diff) {
                // Check ALL orientations
                for (int dth = 0; dth < theta_bins; ++dth) {
                    int chk_th = (grid.gth + dth) % theta_bins;
                    GridKey chk{grid.gx + dx, grid.gy + dy, chk_th};
                    auto it = closed_other.find(chk);
                    if (it != closed_other.end()) {
                        return chk;
                    }
                }
            } else {
                // Ackermann: orientations must closely match
                for (int dth = -2; dth <= 2; ++dth) {
                    int chk_th = ((grid.gth + dth) % theta_bins + theta_bins) % theta_bins;
                    GridKey chk{grid.gx + dx, grid.gy + dy, chk_th};
                    auto it = closed_other.find(chk);
                    if (it != closed_other.end()) {
                        return chk;
                    }
                }
            }
        }
    }
    return std::nullopt;
}

static double euclideanDist(const State& s1, const State& s2, double res) {
    return std::hypot(s1.x - s2.x, s1.y - s2.y) * res;
}

BMHSPlanner::PlanResult BMHSPlanner::plan(const State& start, const State& goal) {
    PlanResult result;
    auto t_start = std::chrono::steady_clock::now();

    int h = map_.height();
    int w = map_.width();
    double res = map_.resolution();

    // ─── 1. Compute dual 2D heuristics in parallel ──────────────────────
    std::cout << "[BMHS] Computing parallel 2D heuristics..." << std::endl;

    int goal_ix = static_cast<int>(goal.x);
    int goal_iy = static_cast<int>(goal.y);
    int start_ix = static_cast<int>(start.x);
    int start_iy = static_cast<int>(start.y);

    // Extract raw pointers from cv::Mat for the Dijkstra function
    const uint8_t* map_ptr = map_.inflatedMap().ptr<uint8_t>(0);
    const float*   clr_ptr = map_.clearanceMap().ptr<float>(0);

    std::vector<double> h2d_goal(h * w);
    std::vector<double> h2d_start(h * w);

    computeBothHeuristics(
        map_ptr, clr_ptr,
        h, w,
        goal_ix, goal_iy,
        start_ix, start_iy,
        res, 1.0,
        h2d_goal.data(), h2d_start.data()
    );

    auto t_heuristic = std::chrono::steady_clock::now();
    double heur_time = std::chrono::duration<double>(t_heuristic - t_start).count();
    std::cout << "[BMHS] Heuristics computed in " << heur_time << "s" << std::endl;

    // ─── 2. Bidirectional kinematic A* search ───────────────────────────
    using PQ = std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>>;

    PQ open_f, open_b;

    std::unordered_map<GridKey, State, GridKeyHash> closed_f, closed_b;
    std::unordered_map<GridKey, double, GridKeyHash> g_f, g_b;

    // came_from: grid → (parent_grid, state_at_this_grid)
    // Using a pair where first = parent grid key, second = state
    struct CameFromEntry { GridKey parent; State state; bool has_parent; };
    std::unordered_map<GridKey, CameFromEntry, GridKeyHash> came_from_f, came_from_b;

    GridKey start_grid = kin_.stateToGrid(start);
    GridKey goal_grid  = kin_.stateToGrid(goal);

    g_f[start_grid] = 0.0;
    g_b[goal_grid]  = 0.0;

    // Forward: heuristic from goal
    double h_f_val = (start_iy >= 0 && start_iy < h && start_ix >= 0 && start_ix < w)
        ? h2d_goal[start_iy * w + start_ix]
        : euclideanDist(start, goal, res);
    open_f.push({weight_ * h_f_val, 0.0, start});
    came_from_f[start_grid] = {GridKey{}, start, false};

    // Backward: heuristic from start
    double h_b_val = (goal_iy >= 0 && goal_iy < h && goal_ix >= 0 && goal_ix < w)
        ? h2d_start[goal_iy * w + goal_ix]
        : euclideanDist(goal, start, res);
    open_b.push({weight_ * h_b_val, 0.0, goal});
    came_from_b[goal_grid] = {GridKey{}, goal, false};

    GridKey intersect_grid_f{}, intersect_grid_b{};
    State   intersect_state_f{}, intersect_state_b{};
    bool    found = false;

    int iters = 0;

    while (!open_f.empty() && !open_b.empty() && iters < max_iterations_) {
        ++iters;

        // ── Expand Forward ──────────────────────────────────────────────
        if (!open_f.empty()) {
            auto top = open_f.top(); open_f.pop();
            GridKey grid_f = kin_.stateToGrid(top.state);

            if (closed_f.find(grid_f) == closed_f.end()) {
                closed_f[grid_f] = top.state;
                result.searched_fwd.push_back(top.state);

                auto chk = checkIntersection(grid_f, closed_b);
                if (chk.has_value()) {
                    intersect_grid_f = grid_f;
                    intersect_grid_b = chk.value();
                    intersect_state_f = top.state;
                    intersect_state_b = closed_b[chk.value()];
                    found = true;
                    break;
                }

                auto prims = kin_.getPrimitives(top.state, map_.clearanceMap(), res, false);
                for (const auto& prim : prims) {
                    if (!map_.isValid(prim.state.x, prim.state.y))
                        continue;

                    GridKey ng = kin_.stateToGrid(prim.state);
                    double new_g = top.g + prim.cost;

                    if (g_f.find(ng) == g_f.end() || new_g < g_f[ng]) {
                        g_f[ng] = new_g;
                        int nix = static_cast<int>(prim.state.x);
                        int niy = static_cast<int>(prim.state.y);
                        double hval = (nix >= 0 && nix < w && niy >= 0 && niy < h)
                            ? h2d_goal[niy * w + nix]
                            : euclideanDist(prim.state, goal, res);

                        open_f.push({new_g + weight_ * hval, new_g, prim.state});
                        came_from_f[ng] = {grid_f, prim.state, true};
                    }
                }
            }
        }

        // ── Expand Backward ─────────────────────────────────────────────
        if (!open_b.empty()) {
            auto top = open_b.top(); open_b.pop();
            GridKey grid_b = kin_.stateToGrid(top.state);

            if (closed_b.find(grid_b) == closed_b.end()) {
                closed_b[grid_b] = top.state;
                result.searched_bwd.push_back(top.state);

                auto chk = checkIntersection(grid_b, closed_f);
                if (chk.has_value()) {
                    intersect_grid_f = chk.value();
                    intersect_grid_b = grid_b;
                    intersect_state_f = closed_f[chk.value()];
                    intersect_state_b = top.state;
                    found = true;
                    break;
                }

                auto prims = kin_.getPrimitives(top.state, map_.clearanceMap(), res, true);
                for (const auto& prim : prims) {
                    if (!map_.isValid(prim.state.x, prim.state.y))
                        continue;

                    GridKey ng = kin_.stateToGrid(prim.state);
                    double new_g = top.g + prim.cost;

                    if (g_b.find(ng) == g_b.end() || new_g < g_b[ng]) {
                        g_b[ng] = new_g;
                        int nix = static_cast<int>(prim.state.x);
                        int niy = static_cast<int>(prim.state.y);
                        double hval = (nix >= 0 && nix < w && niy >= 0 && niy < h)
                            ? h2d_start[niy * w + nix]
                            : euclideanDist(prim.state, start, res);

                        open_b.push({new_g + weight_ * hval, new_g, prim.state});
                        came_from_b[ng] = {grid_b, prim.state, true};
                    }
                }
            }
        }
    }

    auto t_search = std::chrono::steady_clock::now();
    double search_time = std::chrono::duration<double>(t_search - t_heuristic).count();
    std::cout << "[BMHS] Search completed in " << search_time << "s, iterations: " << iters << std::endl;

    if (!found) {
        std::cout << "[BMHS] Failed to find path after " << iters << " iterations." << std::endl;
        result.iterations = iters;
        result.planning_time_s = std::chrono::duration<double>(t_search - t_start).count();
        result.success = false;
        return result;
    }

    std::cout << "[BMHS] Path found after " << iters << " iterations." << std::endl;

    // ─── 3. Reconstruct path ────────────────────────────────────────────
    // Forward part
    std::vector<State> path_f;
    {
        GridKey curr = intersect_grid_f;
        while (came_from_f.find(curr) != came_from_f.end()) {
            auto& entry = came_from_f[curr];
            path_f.push_back(entry.state);
            if (!entry.has_parent) break;
            curr = entry.parent;
        }
        std::reverse(path_f.begin(), path_f.end());
    }

    // Backward part
    std::vector<State> path_b;
    {
        GridKey curr = intersect_grid_b;
        // Include the intersection state for the backward tree
        if (came_from_b.find(curr) != came_from_b.end()) {
            path_b.push_back(intersect_state_b);
            auto& entry = came_from_b[curr];
            if (entry.has_parent) {
                curr = entry.parent;
            } else {
                curr = GridKey{-99999, -99999, -99999}; // sentinel to stop
            }
        }

        while (came_from_b.find(curr) != came_from_b.end()) {
            auto& entry = came_from_b[curr];
            path_b.push_back(entry.state);
            if (!entry.has_parent) break;
            curr = entry.parent;
        }
    }

    // Combine
    std::vector<State> raw_path;
    raw_path.insert(raw_path.end(), path_f.begin(), path_f.end());
    raw_path.insert(raw_path.end(), path_b.begin(), path_b.end());

    std::cout << "[BMHS] Raw path: " << raw_path.size() << " waypoints" << std::endl;

    // ─── 4. Post-processing ─────────────────────────────────────────────
    PathSmoother smoother(map_, kin_);
    auto simplified = smoother.simplifyPath(raw_path);
    std::cout << "[BMHS] Simplified: " << raw_path.size() << " -> " << simplified.size() << " waypoints" << std::endl;

    if (kin_.type() == VehicleKinematics::Type::ACKERMANN && simplified.size() > 2) {
        auto smoothed = smoother.smoothPathAckermann(simplified);
        std::cout << "[BMHS] Ackermann smoothed: " << simplified.size() << " -> " << smoothed.size() << " waypoints" << std::endl;
        result.path = std::move(smoothed);
    } else {
        result.path = std::move(simplified);
    }

    auto t_end = std::chrono::steady_clock::now();
    result.iterations = iters;
    result.planning_time_s = std::chrono::duration<double>(t_end - t_start).count();
    result.success = true;

    std::cout << "[BMHS] Total planning time: " << result.planning_time_s << "s" << std::endl;
    return result;
}

}  // namespace bmhs
