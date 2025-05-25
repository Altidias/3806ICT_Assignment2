#include <turtlebot3_explorer/frontier_detector.h>

FrontierDetector::FrontierDetector(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh)
{
	// Retrieves map topic to listen to
	private_nh.param<std::string>("map_topic", map_topic_, "/map");
	// Gets topic for publishing frontier "centroids"
	private_nh.param<std::string>("frontier_topic", frontier_topic_, "/frontiers");
	// Sets the topic to publish visualisation markers
	private_nh.param<std::string>("frontier_points_topic", frontier_points_topic_, "/frontier_points");
	// Loads the minimum cluster (of cells), and sets it to be 5, ignoring smaller frontier regions
	private_nh.param<int>("min_frontier_size", min_frontier_size_, 5);
	// Not sure if I will use this since I will use adjacency clustering, this loads the
	// minimum clustering distance (0.5m)
	private_nh.param<double>("frontier_cluster_min_dist", frontier_cluster_min_dist_, 0.5);

	// Sub to map_topic_, calling for eahc new map message
	map_sub_ = nh_.subscribe(map_topic_, 1, &FrontierDetector::mapCallback, this);
	// Pub for frontier_topic_ to send PoseArray messages
	frontier_pub_ = nh_.advertise<geometry_msgs::PoseArray>(frontier_topic_, 1);
	// Pub to send MarkerArray messages for RVis visualization
	frontier_points_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(frontier_points_topic_, 1);
	
    
}

// Destructor handled by ROS
FrontierDetector::~FrontierDetector() {
}

// Processor for each new map
void FrontierDetector::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_ = map;
    
    std::vector<Frontier> frontiers = detectFrontiers();
    publishFrontiers(frontiers);
}


std::vector<Frontier> FrontierDetector::detectFrontiers() {
	// Check map has data
	if (!map || map_->data.empty()) {
		return std::vector<Frontier>();
	}

	// x and y coords for each frontier
	std::vector<std::par<int, int>> frontier_points;
	// Reading in map data
	int width = map_->info.width
	int height = map_->info.height;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width, ++x) {
			// Checks if that point is a frontier point
			if (isFrontierPoint(*map_, x, y)) {
				frontier_points.emplace_back(x,y);
			}
		}
	}
	
	// Passes the frontier points to cluserFronteirs to group them into clusters
	std::vector<Frontier> frontiers;
	clusterFrontiers(frontier_points, frontiers);
	// Returns for publishing
	return frontiers;

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
