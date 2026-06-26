#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>
#include <cstdint>

namespace bmhs {

/**
 * @brief Loads a PGM occupancy map, computes binary threshold, inflation,
 *        and clearance (distance transform).
 *
 * All internal data is stored in pixel coordinates.
 * Conversion to/from world coordinates uses resolution and origin.
 */
class MapProcessor {
public:
    MapProcessor() = default;

    /**
     * @brief Load and process a PGM map file.
     * @param map_path      Path to the .pgm file.
     * @param resolution    Meters per pixel.
     * @param vehicle_width Vehicle width in meters (used to compute default inflation).
     * @param vehicle_length Vehicle length in meters.
     * @param inflation_radius Explicit inflation radius in meters (overrides vehicle-based default).
     *                         Set to < 0 to use max(width, length)/2.
     */
    void load(const std::string& map_path,
              double resolution,
              double vehicle_width,
              double vehicle_length,
              double inflation_radius = -1.0);

    /// Check if a pixel coordinate is in free space on the inflated map.
    bool isValid(double x, double y) const;

    // ── Accessors ──────────────────────────────────────────────────────────
    int width()      const { return width_; }
    int height()     const { return height_; }
    double resolution() const { return resolution_; }

    const cv::Mat& rawMap()       const { return raw_map_; }
    const cv::Mat& binaryMap()    const { return binary_map_; }
    const cv::Mat& inflatedMap()  const { return inflated_map_; }
    const cv::Mat& clearanceMap() const { return clearance_map_; }

    // ── ROS2 OccupancyGrid helpers ─────────────────────────────────────────
    /**
     * Convert the raw binary map to OccupancyGrid data (int8 vector).
     * ROS convention: 0 = free, 100 = occupied, -1 = unknown.
     * Handles Y-axis flip (PGM row 0 = top, ROS row 0 = bottom).
     */
    std::vector<int8_t> toOccupancyData() const;

    /**
     * Convert the inflated costmap to OccupancyGrid data.
     * Free cells get a cost based on proximity to obstacles (0-99).
     * Occupied cells get 100.
     */
    std::vector<int8_t> toCostmapData() const;

private:
    cv::Mat raw_map_;        ///< Original grayscale image
    cv::Mat binary_map_;     ///< Thresholded binary (0 or 255)
    cv::Mat inflated_map_;   ///< After morphological erosion
    cv::Mat clearance_map_;  ///< Distance transform (float, pixels)

    int width_  = 0;
    int height_ = 0;
    double resolution_ = 0.05;
};

}  // namespace bmhs
