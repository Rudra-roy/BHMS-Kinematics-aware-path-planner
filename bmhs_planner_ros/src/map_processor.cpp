#include "bmhs_planner_ros/map_processor.hpp"
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace bmhs {

void MapProcessor::load(const std::string& map_path,
                        double resolution,
                        double vehicle_width,
                        double vehicle_length,
                        double inflation_radius) {
    resolution_ = resolution;

    raw_map_ = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
    if (raw_map_.empty()) {
        throw std::runtime_error("Could not read map at: " + map_path);
    }

    // Binary threshold (same as Python: threshold at 200)
    cv::threshold(raw_map_, binary_map_, 200, 255, cv::THRESH_BINARY);

    // Compute inflation radius in pixels
    double inf_rad = inflation_radius;
    if (inf_rad < 0.0) {
        inf_rad = std::max(vehicle_width, vehicle_length) / 2.0;
    }
    int inflation_px = static_cast<int>(std::ceil(inf_rad / resolution_));

    // Inflate obstacles via morphological erosion (free=255, occ=0 → erode shrinks free space)
    if (inflation_px > 0) {
        int kernel_size = inflation_px * 2 + 1;
        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
        cv::erode(binary_map_, inflated_map_, kernel, cv::Point(-1, -1), 1);
    } else {
        inflated_map_ = binary_map_.clone();
    }

    height_ = inflated_map_.rows;
    width_  = inflated_map_.cols;

    // Distance transform for clearance (result in pixels)
    cv::distanceTransform(inflated_map_, clearance_map_, cv::DIST_L2, 5);
}

bool MapProcessor::isValid(double x, double y) const {
    int ix = static_cast<int>(x);
    int iy = static_cast<int>(y);
    if (ix >= 0 && ix < width_ && iy >= 0 && iy < height_) {
        return inflated_map_.at<uint8_t>(iy, ix) == 255;
    }
    return false;
}

std::vector<int8_t> MapProcessor::toOccupancyData() const {
    // ROS OccupancyGrid: row 0 = bottom of map (Y-up).
    // PGM: row 0 = top of map.  We flip Y.
    // Values: 0 = free, 100 = occupied, -1 = unknown.
    std::vector<int8_t> data(width_ * height_);

    for (int row = 0; row < height_; ++row) {
        int ros_row = height_ - 1 - row;  // flip Y
        for (int col = 0; col < width_; ++col) {
            uint8_t pixel = binary_map_.at<uint8_t>(row, col);
            int8_t val;
            if (pixel == 255) {
                val = 0;    // free
            } else if (pixel == 0) {
                val = 100;  // occupied
            } else {
                val = -1;   // unknown / gray
            }
            data[ros_row * width_ + col] = val;
        }
    }
    return data;
}

std::vector<int8_t> MapProcessor::toCostmapData() const {
    // Costmap: inflated map with costs based on clearance.
    // occupied → 100, free but close → scaled 1–99, far → 0.
    std::vector<int8_t> data(width_ * height_);

    // Maximum clearance for cost scaling (in pixels)
    double max_clear_px = 20.0;  // ~1m at 0.05 res

    for (int row = 0; row < height_; ++row) {
        int ros_row = height_ - 1 - row;
        for (int col = 0; col < width_; ++col) {
            uint8_t pixel = inflated_map_.at<uint8_t>(row, col);
            int8_t val;
            if (pixel == 0) {
                val = 100;  // lethal
            } else {
                float clearance = clearance_map_.at<float>(row, col);
                if (clearance < max_clear_px) {
                    // Scale 1–99 (higher cost = closer to obstacles)
                    double ratio = 1.0 - (clearance / max_clear_px);
                    val = static_cast<int8_t>(std::clamp(ratio * 98.0 + 1.0, 0.0, 99.0));
                } else {
                    val = 0;  // free and far from obstacles
                }
            }
            data[ros_row * width_ + col] = val;
        }
    }
    return data;
}

}  // namespace bmhs
