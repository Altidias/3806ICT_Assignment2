#ifndef FRONTIER_DETECTOR_H
#define FRONTIER_DETECTOR_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <vector>
#include <queue>
#include <mutex>

struct Frontier {
    double x, y;  // World coordinates
    int size;     // Number of cells
};

class FrontierDetector {
public:
    FrontierDetector(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~FrontierDetector();

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Publisher frontier_pub_;
    ros::Publisher frontier_points_pub_;
    
    std::string map_topic_;
    std::string frontier_topic_;
    std::string frontier_points_topic_;
    int min_frontier_size_;
    double frontier_cluster_min_dist_;
    
    // map data
    nav_msgs::OccupancyGrid::ConstPtr map_;
    std::mutex map_mutex_;
    
    // detection methods
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    std::vector<Frontier> detectFrontiers();
    void clusterFrontiers(const std::vector<std::pair<int, int>>& frontier_points, std::vector<Frontier>& frontiers);
    bool isFrontierPoint(const nav_msgs::OccupancyGrid& map, int x, int y);
    void publishFrontiers(const std::vector<Frontier>& frontiers);
    
    void mapToWorld(int mx, int my, double& wx, double& wy);
    bool worldToMap(double wx, double wy, int& mx, int& my);
};

#endif // FRONTIER_DETECTOR_H