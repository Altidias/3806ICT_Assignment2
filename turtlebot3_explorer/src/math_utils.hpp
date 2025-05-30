#ifndef UTILS_HPP
#define UTILS_HPP

#include <nav_msgs/OccupancyGrid.h>
#include <cmath>

namespace explorer_utils {

// coordinate conversion stuff
inline void mapToWorld(const nav_msgs::OccupancyGrid::ConstPtr& map, 
                      int mx, int my, double& wx, double& wy) {
    // center of cell not corner
    wx = (mx + 0.5) * map->info.resolution + map->info.origin.position.x;
    wy = (my + 0.5) * map->info.resolution + map->info.origin.position.y;
}

inline bool worldToMap(const nav_msgs::OccupancyGrid::ConstPtr& map,
                      double wx, double wy, int& mx, int& my) {
    if (!map) return false;
    mx = static_cast<int>((wx - map->info.origin.position.x) / map->info.resolution);
    my = static_cast<int>((wy - map->info.origin.position.y) / map->info.resolution);
    
    // bounds check
    return (mx >= 0 && mx < map->info.width && my >= 0 && my < map->info.height);
}

// normalize angle to [-pi, pi]
inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

// manhattan distance for heuristics
inline double manhattanDistance(int x1, int y1, int x2, int y2) {
    return std::abs(x2 - x1) + std::abs(y2 - y1);
}

// euclidean distance
inline double euclideanDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

}  // namespace explorer_utils

#endif  // UTILS_HPP