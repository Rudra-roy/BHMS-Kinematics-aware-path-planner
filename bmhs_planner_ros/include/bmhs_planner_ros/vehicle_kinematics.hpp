#pragma once

#include <vector>
#include <tuple>
#include <cmath>
#include <opencv2/core.hpp>

namespace bmhs {

/// A kinematic state: (x, y, theta) in pixel coordinates.
struct State {
    double x, y, theta;
};

/// A discretised grid key for the state space.
struct GridKey {
    int gx, gy, gth;

    bool operator==(const GridKey& o) const {
        return gx == o.gx && gy == o.gy && gth == o.gth;
    }
};

/// Hash for GridKey so it can be used in unordered_map.
struct GridKeyHash {
    std::size_t operator()(const GridKey& k) const {
        // Combine three ints into a single hash
        std::size_t h = std::hash<int>()(k.gx);
        h ^= std::hash<int>()(k.gy)  + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int>()(k.gth) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

/// A motion primitive result: successor state + step cost.
struct Primitive {
    State state;
    double cost;
};

/**
 * @brief Generates kinematic motion primitives for differential or ackermann
 *        vehicle models.
 *
 * Directly ported from the Python VehicleKinematics class.
 */
class VehicleKinematics {
public:
    enum class Type { DIFFERENTIAL, ACKERMANN };

    /**
     * @param type           Vehicle type.
     * @param turning_radius Physical turning radius in meters (0 for differential).
     * @param resolution     Map resolution in meters/pixel.
     */
    VehicleKinematics(Type type, double turning_radius, double resolution);

    /// Discretise a continuous state into a grid key.
    GridKey stateToGrid(const State& s) const;

    /**
     * @brief Generate all successor motion primitives from a state.
     * @param state         Current state (pixel coords).
     * @param clearance_map Distance-transform map (pixels).
     * @param resolution    Map resolution m/px.
     * @param reverse       If true, generate backward-tree primitives.
     */
    std::vector<Primitive> getPrimitives(const State& state,
                                         const cv::Mat& clearance_map,
                                         double resolution,
                                         bool reverse = false) const;

    Type type()       const { return type_; }
    int  thetaBins()  const { return theta_bins_; }
    double stepSizePx() const { return step_size_px_; }

private:
    Type   type_;
    double tr_px_;          ///< Turning radius in pixels
    double step_size_px_;   ///< Step size in pixels
    int    theta_bins_ = 72;
};

}  // namespace bmhs
