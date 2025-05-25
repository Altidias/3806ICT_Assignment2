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
	private_nh.param<int>("min_frontier_size", min_frontier_size_, 1);
	// Not sure if I will use this since I will use adjacency clustering, this loads the
	// minimum clustering distance (0.5m)
	private_nh.param<double>("frontier_cluster_min_dist", frontier_cluster_min_dist_, 0.5);
	

	// =============== DEBUG ================
	ROS_INFO("Loaded min_frontier_size: %d", min_frontier_size_);
	ROS_INFO("Frontier topic set to: %s", frontier_topic_.c_str());
	// ----------------------------------------

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
	ROS_INFO("Received map message with width: %d, height: %d", map->info.width, map->info.height);
    	map_ = map; 
    	std::vector<Frontier> frontiers = detectFrontiers();
    	ROS_INFO("Detected %lu frontiers in mapCallback", frontiers.size());
    	publishFrontiers(frontiers);
}


std::vector<Frontier> FrontierDetector::detectFrontiers() {
	std::vector<Frontier> frontiers; // Local vector to store frontiers
					
	// Check map has data
	if (!map_ || map_->data.empty()) {
		return frontiers;
	}

	// x and y coords for each frontier
	std::vector<std::pair<int, int>> frontier_points;
	// Reading in map data
	int width = map_->info.width;
	int height = map_->info.height;

	std::lock_guard<std::mutex> lock(map_mutex_);

	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			int idx = y * width + x;
			if (map_->data[idx] != 0) continue; // Skip non-free cells
			// Checks if that point is a frontier point
			if (isFrontierPoint(*map_, x, y)) {
				ROS_INFO("Detected frontier point at (%d, %d)", x, y);
				frontier_points.emplace_back(x,y);
			}
		}
	}
	
	// Passes the frontier points to cluserFronteirs to group them into clusters
	ROS_INFO("Total frontier points detected: %lu", frontier_points.size());
	clusterFrontiers(frontier_points, frontiers);
	// Returns for publishing
	return frontiers;

}

bool FrontierDetector::isFrontierPoint(const nav_msgs::OccupancyGrid& map, int x, int y) {	
	// Get map info
	int width = map.info.width;
	int height = map.info.height;
	// 1D Index for cell
	int index = y * width + x;
	
	// check points are within map limits
	if (x < 0 || x >= width || y < 0 || y >= height) {
		return false;
	}

	// Not a frontier point if it is occupied
	if (map.data[index] != 0) {
		ROS_INFO("Point (%d, %d) not free: value = %d", x, y, map.data[index]);
		return false;
	}
	
	bool has_unexplored_adjacent = false;
	// Loops over a 3x3 section
	for (int i = -1; i <= 1; i++) {
		for (int j = -1; j <= 1; j++){
			// Skips the cell itself, only check 4-adjacent cells 
			// up, down, left, right (ignore diagonals)
			if (j == 0 && i == 0) continue;
			if (j != 0 && i != 0) continue;
			// Coordinates of adjacent neighbours
			int nx = x + j;
			int ny = y + i;
			ROS_INFO("NX: %d, NY %d", nx, ny);
			// Make sure they are within section
			if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
				int n_idx = ny * width + nx;
				ROS_INFO("This run with index: %d", n_idx);
				// If cell is unexplored
				if (map.data[n_idx] == -1) {
					ROS_INFO("DOES THIS EVER RUN\n\n\n\n\n");
					has_unexplored_adjacent = true;
					break;
				}
			}
		}
		if (has_unexplored_adjacent) break;
	}
	// No frontier found
	if (!has_unexplored_adjacent) {
		ROS_INFO("Point (%d, %d) has no unknown neighbors", x, y);
		return false;
	}
	
	// Checking to make sure frontier is reachable within 3x3 grid
	int obstacle_count = 0;
	for (int i = -1; i <= 1; i++) {
		for (int j = -1; j <- 1; j++) {
			int nx = x + j;
			int ny = y + i;
			if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
				int n_idx = ny * width * nx;
				// If there is an obstacle (idx value of 100 within map)
				if (map.data[n_idx] == 100) {
					obstacle_count++;
				}
			}
		}
	
	}
	// Allow for 4 obstacles
	if (obstacle_count > 4) {
        ROS_INFO("Point (%d, %d) has too many obstacles nearby: %d", x, y, obstacle_count);
        return false;
    	}

	ROS_INFO("Point (%d, %d) is a frontier", x, y);
	return true;	
}

// Flood-fill algorithm
void FrontierDetector::clusterFrontiers(const std::vector<std::pair<int, int>>& frontier_points, std::vector<Frontier>& frontiers) {
	// Vector for visited points
	std::vector<bool> visited(frontier_points.size(), false);
	// Map info
	int width = map_->info.width;
	int height = map_->info.height;
	
	// This will detect cluster using the flood-fill algorithm
	// Goes over frontier points, skipping the already visited one
	for (size_t i = 0; i < frontier_points.size(); ++i) {
		if (visited[i]) continue;
		
		// Stores the clusters	
		std::vector<std::pair<int, int>> cluster;
		// Stack to manage the flood-fill process
		std::vector<size_t> stack;
		
		// Start point is marked as visited
		stack.push_back(i);
		visited[i] = true;
	
		// As long as there are points to explore in the current cluster 
		// DFS to connect adjacent frontier points
		while (!stack.empty()) {
			// Takes stack last index and adds coords to cluster
			// This builds the cluster one point at a time
			size_t current = stack.back();
			stack.pop_back();
			cluster.push_back(frontier_points[current]);
			
			// Store  and y grid coords of current point for future comparison
			int x = frontier_points[current].first;
			int y = frontier_points[current].second;
			
			// Loops over all frontier points
			// Looks for unvisited neighbours of current point
			for (size_t j = 0; j < frontier_points.size(); ++j) {
				// Skip if visited
				if (visited[j]) continue;
				
				// Coords of neighbour
				int nx = frontier_points[j].first;
				int ny = frontier_points[j].second;
					
				// Check if neighbour is xy adjacent (not diagonal)
				// If it is, neighbours index is add to stack 
				// Marked as visited
				if ((std::abs(nx - x) == 1 && ny == y) || 
				(std::abs(ny - y) == 1 && nx == x)) {
					stack.push_back(j);
					visited[j] = true;
				}
			}
		}

		// The next section processes the clusters we found
		// It also calculates the centroids
		// First, check if min_frontier_size_ is met to reduce noise
		if (static_cast<int>(cluster.size()) >= min_frontier_size_) {
			double avg_x = 0.0, avg_y = 0.0;
			
			// Calc the points average position within each cluster
			for (const auto& point : cluster) {
				avg_x += point.first;
				avg_y += point.second;
			}	
			avg_x /= cluster.size();
			avg_y /= cluster.size();
			

			double wx, wy;
			// Convert the centroid's grid coords to world coords
			mapToWorld(static_cast<int>(avg_x), static_cast<int>(avg_y), wx, wy);

			// Adds that frontier to the vector
			Frontier frontier;

			frontier.x = wx;
			frontier.y = wy;

			frontier.size = cluster.size();
			frontiers.push_back(frontier);
		}
	}
	
	// This section is the visualisation for RVis
	visualization_msgs::MarkerArray marker_array;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "frontier_points";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;
	marker.lifetime = ros::Duration(2.0);
	
	// Convert frontier_points to world coords
	for (const auto& point : frontier_points) {
		double wx, wy;

		mapToWorld(point.first, point.second, wx, wy);
		geometry_msgs::Point p;
		p.x = wx;
		p.y = wy;
		p.z = 0.0;
		marker.points.push_back(p);
	}

	// Publish marker array and publish to /frontier_points_topic_ (/front_points)
	// RVis will display things as red dots
	marker_array.markers.push_back(marker);
	frontier_points_pub_.publish(marker_array);
}

// Publishing frontier clusters to PoseArray for nav
void FrontierDetector::publishFrontiers(const std::vector<Frontier>& frontiers) {
	// Holds frontier centroids
	geometry_msgs::PoseArray frontier_arr; 
	// Attacked timestamp to map
	frontier_arr.header.stamp = ros::Time::now();
	frontier_arr.header.frame_id = "map";
	
	// Goes through each Frontier object
	for (const auto& frontier : frontiers) {
		// Creates a Pose from frontier coords
		geometry_msgs::Pose pose;
		pose.position.x = frontier.x;
		pose.position.y = frontier.y;
		pose.position.z = 0.0;
		pose.orientation.w = 1.0;
	
		// Add pose to array 
		frontier_arr.poses.push_back(pose);
	}
	// Publishes PoseArray to frontier_topic_ (/frontiers)
	frontier_pub_.publish(frontier_arr);

}

// Converts grid coordinates to world coords
void FrontierDetector::mapToWorld(int mx, int my, double& wx, double& wy) {
	if (!map_) return;
	wx = map_->info.origin.position.x + (mx + 0.5) * map_->info.resolution;
	wy - map_->info.origin.position.y + (my + 0.5) * map_->info.resolution;
}

// Converts world coords to grid coords
bool FrontierDetector::worldToMap(double wx, double wy, int& mx, int& my) {
	if (!map_) return false;

	mx = static_cast<int>((wx - map_->info.origin.position.x) / map_->info.resolution);
	my = static_cast<int>((wy - map_->info.origin.position.y) / map_->info.resolution);
	if (mx < 0 || mx >= static_cast<int>(map_->info.width) ||
		my < 0 || my >= static_cast<int>(map_->info.height)) {
		return false;
	}
	return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "frontier_detector");
    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    FrontierDetector detector(nh, private_nh);
    
    ros::spin();
    
    return 0;
}




