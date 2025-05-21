#include <turtlebot3_explorer/frontier_detector.h>

FrontierDetector::FrontierDetector(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh)
{
    
}

FrontierDetector::~FrontierDetector() {
}

void FrontierDetector::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_ = map;
    
    std::vector<Frontier> frontiers = detectFrontiers();
    publishFrontiers(frontiers);
}

std::vector<Frontier> FrontierDetector::detectFrontiers() {
    
}

bool FrontierDetector::isFrontierPoint(const nav_msgs::OccupancyGrid& map, int x, int y) {
    
}

void FrontierDetector::clusterFrontiers(const std::vector<std::pair<int, int>>& frontier_points, std::vector<Frontier>& frontiers) {
    
}

void FrontierDetector::publishFrontiers(const std::vector<Frontier>& frontiers) {
    
}

void FrontierDetector::mapToWorld(int mx, int my, double& wx, double& wy) {
    
}

bool FrontierDetector::worldToMap(double wx, double wy, int& mx, int& my) {
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "frontier_detector");
    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    FrontierDetector detector(nh, private_nh);
    
    ros::spin();
    
    return 0;
}