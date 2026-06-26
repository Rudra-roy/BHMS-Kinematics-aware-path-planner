#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <queue>
#include <vector>
#include <cmath>
#include <thread>
#include <limits>

namespace py = pybind11;

// ============================================================================
// 2D Dijkstra with clearance penalty - raw pointer implementation
// This is the performance-critical function that replaces the 15-second
// Python heapq-based Dijkstra.
// ============================================================================

struct DijkstraEntry {
    double dist;
    int x, y;

    bool operator>(const DijkstraEntry& other) const {
        return dist > other.dist;
    }
};

static void dijkstra_with_clearance(
    const uint8_t* map_data,
    const float* clearance_data,
    double* dist_data,
    int height, int width,
    int target_x, int target_y,
    double resolution,
    double clearance_threshold
) {
    const double INF = std::numeric_limits<double>::infinity();
    const int total = height * width;

    // Initialize all distances to infinity
    for (int i = 0; i < total; i++)
        dist_data[i] = INF;

    // Bounds check
    if (target_x < 0 || target_x >= width || target_y < 0 || target_y >= height)
        return;
    if (map_data[target_y * width + target_x] != 255)
        return;

    // Min-heap priority queue
    std::priority_queue<DijkstraEntry, std::vector<DijkstraEntry>,
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

        for (int i = 0; i < 8; i++) {
            const int nx = top.x + dx[i];
            const int ny = top.y + dy[i];

            if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                continue;

            const int nidx = ny * width + nx;
            if (map_data[nidx] != 255)
                continue;

            // Clearance penalty: penalize cells close to obstacles
            const double clearance = clearance_data[nidx] * resolution;
            const double penalty = (clearance < clearance_threshold)
                ? (1.0 / (clearance + 0.1))
                : 0.0;
            const double new_d = top.dist + dc[i] + penalty;

            if (new_d < dist_data[nidx]) {
                dist_data[nidx] = new_d;
                pq.push({new_d, nx, ny});
            }
        }
    }

    // Scale accumulated pixel-unit distances by resolution to get meters
    for (int i = 0; i < total; i++) {
        if (dist_data[i] < INF)
            dist_data[i] *= resolution;
    }
}

// ============================================================================
// Python-facing: compute a single 2D heuristic
// ============================================================================

py::array_t<double> compute_2d_heuristic(
    py::array_t<uint8_t, py::array::c_style | py::array::forcecast> inflated_map,
    py::array_t<float, py::array::c_style | py::array::forcecast> clearance_map,
    int target_x, int target_y,
    double resolution,
    double clearance_threshold
) {
    auto map_buf = inflated_map.request();
    auto clear_buf = clearance_map.request();

    const int h = static_cast<int>(map_buf.shape[0]);
    const int w = static_cast<int>(map_buf.shape[1]);

    auto result = py::array_t<double>({h, w});

    dijkstra_with_clearance(
        static_cast<const uint8_t*>(map_buf.ptr),
        static_cast<const float*>(clear_buf.ptr),
        static_cast<double*>(result.request().ptr),
        h, w, target_x, target_y, resolution, clearance_threshold
    );

    return result;
}

// ============================================================================
// Python-facing: compute BOTH heuristics in parallel using C++ threads
// This bypasses Python's GIL entirely for true CPU parallelism.
// ============================================================================

py::tuple compute_both_heuristics(
    py::array_t<uint8_t, py::array::c_style | py::array::forcecast> inflated_map,
    py::array_t<float, py::array::c_style | py::array::forcecast> clearance_map,
    int goal_x, int goal_y,
    int start_x, int start_y,
    double resolution,
    double clearance_threshold
) {
    auto map_buf = inflated_map.request();
    auto clear_buf = clearance_map.request();

    const int h = static_cast<int>(map_buf.shape[0]);
    const int w = static_cast<int>(map_buf.shape[1]);

    // Extract raw pointers (read-only inputs shared between threads)
    const uint8_t* map_ptr = static_cast<const uint8_t*>(map_buf.ptr);
    const float* clear_ptr = static_cast<const float*>(clear_buf.ptr);

    // Allocate output numpy arrays (each thread writes to its own array)
    auto h_goal = py::array_t<double>({h, w});
    auto h_start = py::array_t<double>({h, w});

    double* h_goal_ptr = static_cast<double*>(h_goal.request().ptr);
    double* h_start_ptr = static_cast<double*>(h_start.request().ptr);

    {
        // Release Python's GIL so both C++ threads run truly in parallel
        py::gil_scoped_release release;

        std::thread t1(dijkstra_with_clearance,
            map_ptr, clear_ptr, h_goal_ptr,
            h, w, goal_x, goal_y, resolution, clearance_threshold);

        std::thread t2(dijkstra_with_clearance,
            map_ptr, clear_ptr, h_start_ptr,
            h, w, start_x, start_y, resolution, clearance_threshold);

        t1.join();
        t2.join();
    }

    return py::make_tuple(h_goal, h_start);
}

// ============================================================================
// Pybind11 module definition
// ============================================================================

PYBIND11_MODULE(bmhs_core, m) {
    m.doc() = "C++ accelerated BMHS path planning core - replaces Python Dijkstra";

    m.def("compute_2d_heuristic", &compute_2d_heuristic,
          "Compute 2D Dijkstra heuristic with clearance penalty for a single target",
          py::arg("inflated_map"), py::arg("clearance_map"),
          py::arg("target_x"), py::arg("target_y"),
          py::arg("resolution"),
          py::arg("clearance_threshold") = 1.0);

    m.def("compute_both_heuristics", &compute_both_heuristics,
          "Compute both goal and start heuristics in parallel using C++ threads",
          py::arg("inflated_map"), py::arg("clearance_map"),
          py::arg("goal_x"), py::arg("goal_y"),
          py::arg("start_x"), py::arg("start_y"),
          py::arg("resolution"),
          py::arg("clearance_threshold") = 1.0);
}
