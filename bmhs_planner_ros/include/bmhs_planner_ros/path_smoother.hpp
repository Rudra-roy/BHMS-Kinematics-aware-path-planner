#pragma once

#include "bmhs_planner_ros/map_processor.hpp"
#include "bmhs_planner_ros/vehicle_kinematics.hpp"
#include <vector>

namespace bmhs {

/**
 * @brief Post-processing utilities for planned paths.
 *
 * - simplifyPath():  Remove redundant waypoints via Bresenham line-of-sight
 *                    with clearance enforcement.
 * - smoothPathAckermann():  Replace sharp corners with quadratic Bezier arcs
 *                           that respect the minimum turning radius.
 */
class PathSmoother {
public:
    /**
     * @param map_proc Reference to the processed map (for validity / clearance checks).
     * @param kin      Vehicle kinematics (for turning radius in Bezier smoothing).
     */
    PathSmoother(const MapProcessor& map_proc, const VehicleKinematics& kin);

    /**
     * @brief Simplify a path by removing unnecessary intermediate waypoints.
     *
     * Uses Bresenham line-of-sight with a minimum clearance constraint.
     * If waypoint A can see waypoint C directly with safe clearance, waypoint B
     * is removed.
     */
    std::vector<State> simplifyPath(const std::vector<State>& path) const;

    /**
     * @brief Smooth an Ackermann path by inserting quadratic Bezier curves
     *        at sharp corners.
     *
     * Only applicable for ackermann vehicles.  The curves are constrained to
     * maintain minimum clearance and stay in free space.
     */
    std::vector<State> smoothPathAckermann(const std::vector<State>& path) const;

private:
    /**
     * @brief Bresenham line check from (x0,y0) to (x1,y1).
     * @return true if the entire line is free AND has minimum clearance.
     */
    bool hasLineOfSight(double x0, double y0, double x1, double y1,
                        double min_clearance_m = 0.5) const;

    const MapProcessor& map_;
    const VehicleKinematics& kin_;
};

}  // namespace bmhs
