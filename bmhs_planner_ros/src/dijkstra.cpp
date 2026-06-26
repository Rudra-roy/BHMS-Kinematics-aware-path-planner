#include "bmhs_planner_ros/dijkstra.hpp"

#include <queue>
#include <vector>
#include <cmath>
#include <thread>
#include <limits>

namespace bmhs {

// ============================================================================
// 2D Dijkstra with clearance penalty — raw pointer implementation
// Identical algorithm to the original bmhs_core.cpp, minus pybind11 wrappers.
// ============================================================================

struct DijkstraEntry {
    double dist;
    int x, y;

    bool operator>(const DijkstraEntry& other) const {
        return dist > other.dist;
    }
};

void dijkstraWithClearance(
    const uint8_t* map_data,
    const float*   clearance_data,
    double*        dist_data,
    int height, int width,
    int target_x, int target_y,
    double resolution,
    double clearance_thresh)
{
    const double INF = std::numeric_limits<double>::infinity();
    const int total = height * width;

    // Initialise all distances to infinity
    for (int i = 0; i < total; ++i)
        dist_data[i] = INF;

    // Bounds check
    if (target_x < 0 || target_x >= width || target_y < 0 || target_y >= height)
        return;
    if (map_data[target_y * width + target_x] != 255)
        return;

    // Min-heap priority queue
    std::priority_queue<DijkstraEntry,
                        std::vector<DijkstraEntry>,
                        std::greater<DijkstraEntry>> pq;

    dist_data[target_y * width + target_x] = 0.0;
    pq.push({0.0, target_x, target_y});

    // 8-connected grid directions: dx, dy, base_cost
    static const int dx[] = {1, -1, 0, 0, 1, -1, 1, -1};
    static const int dy[] = {0, 0, 1, -1, 1, 1, -1, -1};
    static const double dc[] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

    while (!pq.empty()) {
        auto top = pq.top();
        pq.pop();

        const int idx = top.y * width + top.x;
        if (top.dist > dist_data[idx])
            continue;

        for (int i = 0; i < 8; ++i) {
            const int nx = top.x + dx[i];
            const int ny = top.y + dy[i];

            if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                continue;

            const int nidx = ny * width + nx;
            if (map_data[nidx] != 255)
                continue;

            // Clearance penalty: penalise cells close to obstacles
            const double clearance = clearance_data[nidx] * resolution;
            const double penalty = (clearance < clearance_thresh)
                ? (1.0 / (clearance + 0.1))
                : 0.0;
            const double new_d = top.dist + dc[i] + penalty;

            if (new_d < dist_data[nidx]) {
                dist_data[nidx] = new_d;
                pq.push({new_d, nx, ny});
            }
        }
    }

    // Scale accumulated pixel-unit distances by resolution to get metres
    for (int i = 0; i < total; ++i) {
        if (dist_data[i] < INF)
            dist_data[i] *= resolution;
    }
}

void computeBothHeuristics(
    const uint8_t* map_data,
    const float*   clearance_data,
    int height, int width,
    int goal_x,  int goal_y,
    int start_x, int start_y,
    double resolution,
    double clearance_thresh,
    double* h_goal,
    double* h_start)
{
    // Run both Dijkstra searches truly in parallel using std::thread
    std::thread t1(dijkstraWithClearance,
        map_data, clearance_data, h_goal,
        height, width, goal_x, goal_y, resolution, clearance_thresh);

    std::thread t2(dijkstraWithClearance,
        map_data, clearance_data, h_start,
        height, width, start_x, start_y, resolution, clearance_thresh);

    t1.join();
    t2.join();
}

}  // namespace bmhs
