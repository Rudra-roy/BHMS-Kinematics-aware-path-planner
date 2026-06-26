#include "bmhs_planner_ros/vehicle_kinematics.hpp"
#include <algorithm>

namespace bmhs {

VehicleKinematics::VehicleKinematics(Type type, double turning_radius, double resolution)
    : type_(type)
    , tr_px_(turning_radius / resolution)
    , step_size_px_(std::max(2.0, 0.25 / resolution))
    , theta_bins_(72)
{
}

GridKey VehicleKinematics::stateToGrid(const State& s) const {
    // Wrap theta to [0, 2π)
    double th = std::fmod(s.theta + 2.0 * M_PI, 2.0 * M_PI);
    int th_bin = static_cast<int>(std::round(th / (2.0 * M_PI / theta_bins_))) % theta_bins_;

    // Bin x and y by a fraction of step size to allow intersection between trees
    double bin_size = std::max(1.0, step_size_px_ * 0.5);
    int gx = static_cast<int>(s.x / bin_size);
    int gy = static_cast<int>(s.y / bin_size);

    return {gx, gy, th_bin};
}

std::vector<Primitive> VehicleKinematics::getPrimitives(
    const State& state,
    const cv::Mat& clearance_map,
    double resolution,
    bool reverse) const
{
    std::vector<Primitive> primitives;
    double x = state.x;
    double y = state.y;
    double theta = state.theta;

    if (type_ == Type::ACKERMANN) {
        double max_delta_theta = (tr_px_ > 0.0)
            ? (step_size_px_ / tr_px_)
            : 0.5;

        double steerings[] = {0.0, max_delta_theta, -max_delta_theta};
        int directions[] = {1, -1};

        for (int d : directions) {
            for (double steer : steerings) {
                int real_d = reverse ? -d : d;

                double new_theta = theta + real_d * steer;
                // Wrap to [-π, π)
                new_theta = std::fmod(new_theta + M_PI, 2.0 * M_PI) - M_PI;

                double new_x = x + real_d * step_size_px_ * std::cos(theta + real_d * steer / 2.0);
                double new_y = y + real_d * step_size_px_ * std::sin(theta + real_d * steer / 2.0);

                double cost = step_size_px_ * resolution;

                // Clearance penalty
                int ix = static_cast<int>(new_x);
                int iy = static_cast<int>(new_y);
                if (ix >= 0 && ix < clearance_map.cols &&
                    iy >= 0 && iy < clearance_map.rows) {
                    double clearance = clearance_map.at<float>(iy, ix) * resolution;
                    if (clearance < 0.5) {
                        cost += 1.5 / (clearance + 0.1);
                    }
                }

                if (steer != 0.0) {
                    cost += 1.0 * std::abs(steer);
                }
                if (d == -1) {
                    cost += 5.0 * step_size_px_ * resolution;
                }

                primitives.push_back({{new_x, new_y, new_theta}, cost});
            }
        }
    } else {
        // Differential drive: composite "turn-then-move" primitives
        // 8 forward directions cover all 360°
        for (int i = 0; i < 8; ++i) {
            double target_theta = i * M_PI / 4.0;

            // Normalise angle difference to [-π, π]
            double angle_diff = std::fmod(target_theta - theta + M_PI, 2.0 * M_PI) - M_PI;

            int real_d = reverse ? -1 : 1;

            double new_x = x + real_d * step_size_px_ * std::cos(target_theta);
            double new_y = y + real_d * step_size_px_ * std::sin(target_theta);

            // Base movement cost
            double cost = step_size_px_ * resolution;

            // Turn cost: penalise heading change to reward straight paths
            if (std::abs(angle_diff) > 0.01) {
                cost += 2.0 + 3.0 * std::abs(angle_diff);
            }

            // Clearance penalty
            int nix = static_cast<int>(new_x);
            int niy = static_cast<int>(new_y);
            if (nix >= 0 && nix < clearance_map.cols &&
                niy >= 0 && niy < clearance_map.rows) {
                double clearance = clearance_map.at<float>(niy, nix) * resolution;
                if (clearance < 0.5) {
                    cost += 1.5 / (clearance + 0.1);
                }
            }

            double new_theta = std::fmod(target_theta + M_PI, 2.0 * M_PI) - M_PI;
            primitives.push_back({{new_x, new_y, new_theta}, cost});
        }
    }

    return primitives;
}

}  // namespace bmhs
