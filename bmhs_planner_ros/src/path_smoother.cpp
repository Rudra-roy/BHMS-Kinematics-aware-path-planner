#include "bmhs_planner_ros/path_smoother.hpp"
#include <cmath>
#include <algorithm>

namespace bmhs {

PathSmoother::PathSmoother(const MapProcessor& map_proc, const VehicleKinematics& kin)
    : map_(map_proc)
    , kin_(kin)
{
}

// ============================================================================
// Bresenham line-of-sight with clearance enforcement
// ============================================================================

bool PathSmoother::hasLineOfSight(double x0, double y0, double x1, double y1,
                                   double min_clearance_m) const {
    int ix0 = static_cast<int>(x0), iy0 = static_cast<int>(y0);
    int ix1 = static_cast<int>(x1), iy1 = static_cast<int>(y1);

    const cv::Mat& clr = map_.clearanceMap();
    double res = map_.resolution();

    int dx = std::abs(ix1 - ix0);
    int dy = std::abs(iy1 - iy0);
    int sx = (ix0 < ix1) ? 1 : -1;
    int sy = (iy0 < iy1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (!map_.isValid(ix0, iy0))
            return false;

        if (ix0 >= 0 && ix0 < clr.cols && iy0 >= 0 && iy0 < clr.rows) {
            if (clr.at<float>(iy0, ix0) * res < min_clearance_m)
                return false;
        }

        if (ix0 == ix1 && iy0 == iy1)
            break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            ix0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            iy0 += sy;
        }
    }

    return true;
}

// ============================================================================
// Path simplification
// ============================================================================

std::vector<State> PathSmoother::simplifyPath(const std::vector<State>& path) const {
    if (path.empty() || path.size() <= 2)
        return path;

    std::vector<State> simplified;
    simplified.push_back(path[0]);
    size_t i = 0;

    while (i < path.size() - 1) {
        size_t farthest = i + 1;
        for (size_t j = path.size() - 1; j > i; --j) {
            if (hasLineOfSight(path[i].x, path[i].y, path[j].x, path[j].y)) {
                farthest = j;
                break;
            }
        }

        // Update heading of the last added point to face the next waypoint
        double dx = path[farthest].x - path[i].x;
        double dy = path[farthest].y - path[i].y;
        double heading = std::atan2(dy, dx);

        simplified.back().theta = heading;
        simplified.push_back({path[farthest].x, path[farthest].y, path[farthest].theta});
        i = farthest;
    }

    return simplified;
}

// ============================================================================
// Ackermann Bezier smoothing
// ============================================================================

std::vector<State> PathSmoother::smoothPathAckermann(const std::vector<State>& path) const {
    if (path.size() <= 2)
        return path;

    // Use step size as a proxy for turning radius (actual tr_px is private)
    double tr_px = kin_.stepSizePx() * 2.0;

    std::vector<State> smoothed;
    smoothed.push_back(path[0]);

    for (size_t i = 1; i < path.size() - 1; ++i) {
        const State& prev = path[i - 1];
        const State& curr = path[i];
        const State& next = path[i + 1];

        double dx_in  = curr.x - prev.x;
        double dy_in  = curr.y - prev.y;
        double dx_out = next.x - curr.x;
        double dy_out = next.y - curr.y;

        double len_in  = std::hypot(dx_in, dy_in);
        double len_out = std::hypot(dx_out, dy_out);

        if (len_in < 1e-6 || len_out < 1e-6) {
            smoothed.push_back(curr);
            continue;
        }

        // How far from the corner to start/end the arc
        double offset = std::min({tr_px * 3.0, len_in * 0.4, len_out * 0.4});

        // Entry point (on incoming segment, before corner)
        double entry_x = curr.x - (dx_in / len_in) * offset;
        double entry_y = curr.y - (dy_in / len_in) * offset;

        // Exit point (on outgoing segment, after corner)
        double exit_x = curr.x + (dx_out / len_out) * offset;
        double exit_y = curr.y + (dy_out / len_out) * offset;

        double theta_in = std::atan2(dy_in, dx_in);
        smoothed.push_back({entry_x, entry_y, theta_in});

        // Generate quadratic Bezier: entry → corner (control) → exit
        int num_samples = std::max(10, static_cast<int>(offset / 2.0));
        bool curve_valid = true;
        std::vector<State> curve_points;

        for (int t_idx = 1; t_idx <= num_samples; ++t_idx) {
            double t = static_cast<double>(t_idx) / num_samples;

            // Quadratic Bezier formula
            double x = (1 - t) * (1 - t) * entry_x + 2 * (1 - t) * t * curr.x + t * t * exit_x;
            double y = (1 - t) * (1 - t) * entry_y + 2 * (1 - t) * t * curr.y + t * t * exit_y;

            // Tangent for heading
            double tx = 2 * (1 - t) * (curr.x - entry_x) + 2 * t * (exit_x - curr.x);
            double ty = 2 * (1 - t) * (curr.y - entry_y) + 2 * t * (exit_y - curr.y);
            double theta = std::atan2(ty, tx);

            if (!map_.isValid(x, y)) {
                curve_valid = false;
                break;
            }

            int ix = static_cast<int>(x);
            int iy = static_cast<int>(y);
            if (ix >= 0 && ix < map_.width() && iy >= 0 && iy < map_.height()) {
                double clearance = map_.clearanceMap().at<float>(iy, ix) * map_.resolution();
                if (clearance < 0.5) {
                    curve_valid = false;
                    break;
                }
            }

            curve_points.push_back({x, y, theta});
        }

        if (curve_valid && !curve_points.empty()) {
            smoothed.insert(smoothed.end(), curve_points.begin(), curve_points.end());
        } else {
            // Fallback: keep the original corner
            smoothed.push_back(curr);
        }
    }

    smoothed.push_back(path.back());
    return smoothed;
}

}  // namespace bmhs
