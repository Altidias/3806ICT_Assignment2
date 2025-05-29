#ifndef FRONTIER_DETECTOR_H
#define FRONTIER_DETECTOR_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <vector>
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
    ros::Publisher frontier_pub_;         // centroids as PoseArray
    ros::Publisher frontier_points_pub_;  // all points as PoseArray
    ros::Publisher frontier_viz_pub_;     // visualization markers
    
    std::string map_topic_;
    std::string frontier_topic_;
    std::string frontier_points_topic_;
    int min_frontier_size_;
    double frontier_cluster_min_dist_;
    
    nav_msgs::OccupancyGrid::ConstPtr map_;
    
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void detectFrontiers(std::vector<Frontier>& frontiers, 
                        std::vector<std::pair<int, int>>& all_frontier_points);
    bool isFrontierPoint(const nav_msgs::OccupancyGrid& map, int x, int y);
    void clusterFrontiers(const std::vector<std::pair<int, int>>& frontier_points,
                         std::vector<Frontier>& frontiers);
    
    void publishFrontierCentroids(const std::vector<Frontier>& frontiers);
    void publishFrontierPoints(const std::vector<std::pair<int, int>>& points);
    void publishVisualizationMarkers(const std::vector<Frontier>& frontiers,
                                   const std::vector<std::pair<int, int>>& points);
    
    void mapToWorld(int mx, int my, double& wx, double& wy);
    bool worldToMap(double wx, double wy, int& mx, int& my);
};

#endif // FRONTIER_DETECTOR_H