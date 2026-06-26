#pragma once

#include <vector>
#include <cstdint>

namespace bmhs {

/**
 * @brief 2D Dijkstra with clearance penalty on an 8-connected grid.
 *
 * Ported from bmhs_core.cpp — identical algorithm, stripped of pybind11.
 * Computes shortest-path distances from a target cell to every reachable cell,
 * penalising cells that are close to obstacles.
 *
 * @param map_data         Row-major uint8 inflated map (255 = free, 0 = occupied).
 * @param clearance_data   Row-major float distance-transform (pixel units).
 * @param[out] dist_data   Output: row-major distances in metres.  Must be pre-allocated (h*w).
 * @param height           Map height in pixels.
 * @param width            Map width in pixels.
 * @param target_x         Target column (pixel).
 * @param target_y         Target row (pixel).
 * @param resolution       Metres per pixel.
 * @param clearance_thresh Clearance threshold in metres below which penalty applies.
 */
void dijkstraWithClearance(
    const uint8_t* map_data,
    const float*   clearance_data,
    double*        dist_data,
    int height, int width,
    int target_x, int target_y,
    double resolution,
    double clearance_thresh = 1.0
);

/**
 * @brief Compute dual 2D heuristics in parallel using std::thread.
 *
 * @param map_data       Inflated map (row-major uint8).
 * @param clearance_data Distance transform (row-major float).
 * @param height         Map height.
 * @param width          Map width.
 * @param goal_x, goal_y Goal cell for heuristic 1 (guides forward tree).
 * @param start_x, start_y Start cell for heuristic 2 (guides backward tree).
 * @param resolution     Metres/pixel.
 * @param clearance_thresh Clearance threshold.
 * @param[out] h_goal    Output heuristic from goal (pre-allocated h*w).
 * @param[out] h_start   Output heuristic from start (pre-allocated h*w).
 */
void computeBothHeuristics(
    const uint8_t* map_data,
    const float*   clearance_data,
    int height, int width,
    int goal_x,  int goal_y,
    int start_x, int start_y,
    double resolution,
    double clearance_thresh,
    double* h_goal,
    double* h_start
);

}  // namespace bmhs
