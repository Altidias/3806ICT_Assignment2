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
	private_nh.param<int>("min_frontier_size", min_frontier_size_, 10);
	// Not sure if I will use this since I will use adjacency clustering, this loads the
	// minimum clustering distance (0.5m)
	private_nh.param<double>("frontier_cluster_min_dist", frontier_cluster_min_dist_, 1.5);
	

	// =============== DEBUG ================
	ROS_INFO("Frontier Detector initialized:");
    ROS_INFO("  Map topic: %s", map_topic_.c_str());
    ROS_INFO("  Frontier centroids topic: %s", frontier_topic_.c_str());
    ROS_INFO("  Frontier points topic: %s", frontier_points_topic_.c_str());
    ROS_INFO("  Min frontier size: %d", min_frontier_size_);
	// ----------------------------------------

	// Sub to map_topic_, calling for eahc new map message
	map_sub_ = nh_.subscribe(map_topic_, 1, &FrontierDetector::mapCallback, this);
	// Pub for frontier_topic_ to send PoseArray messages, publishes frontier centroids
	frontier_pub_ = nh_.advertise<geometry_msgs::PoseArray>(frontier_topic_, 10, true);
    // all frontier points
    frontier_points_pub_ = nh_.advertise<geometry_msgs::PoseArray>(frontier_points_topic_, 10, true);
	// Pub to send MarkerArray messages for RVis visualization
	frontier_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/frontier_markers", 1);


    
}


// Destructor handled by ROS
FrontierDetector::~FrontierDetector() {
}

// Processor for each new map
void FrontierDetector::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
	ROS_INFO("Received map message with width: %d, height: %d", map->info.width, map->info.height);
    map_ = map;
    
    std::vector<Frontier> frontiers;
    std::vector<std::pair<int, int>> all_frontier_points;
    detectFrontiers(frontiers, all_frontier_points);
    
    // all three
    publishFrontierCentroids(frontiers);
    publishFrontierPoints(all_frontier_points);
    publishVisualizationMarkers(frontiers, all_frontier_points);
}


void FrontierDetector::detectFrontiers(std::vector<Frontier>& frontiers, 
    std::vector<std::pair<int, int>>& all_frontier_points) {
					
	// Check map has data
	if (!map_ || map_->data.empty()) {
		return;
	}

	// Reading in map data
	int width = map_->info.width;
	int height = map_->info.height;

	for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (isFrontierPoint(*map_, x, y)) {
                all_frontier_points.emplace_back(x, y);
            }
        }
    }

    clusterFrontiers(all_frontier_points, frontiers);
	
	ROS_INFO_THROTTLE(5.0, "Detected %lu frontier points in %lu clusters", 
                      all_frontier_points.size(), frontiers.size());
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
		return false;
	}
	
	bool has_unexplored_adjacent = false;
	// Loops over a 3x3 section
	for (int i = -1; i <= 1; i++) {
		for (int j = -1; j <= 1; j++){
			// Skips the cell itself, only check 4-adjacent cells 
			// up, down, left, right (ignore diagonals)
			if (j == 0 && i == 0) continue;

			// Coordinates of adjacent neighbours
			int nx = x + j;
			int ny = y + i;
			// Make sure they are within section
			if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
				int n_idx = ny * width + nx;
				// If cell is unexplored
				if (map.data[n_idx] == -1) {
					has_unexplored_adjacent = true;
					break;
				}
			}
		}
		if (has_unexplored_adjacent) break;
	}
	return has_unexplored_adjacent;
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
			ROS_INFO("Clustered frontier at (wx=%f, wy=%f) with size %lu", wx, wy, cluster.size());
		} else {
            		ROS_INFO("Cluster discarded: size %lu < min_frontier_size_ %d", cluster.size(), min_frontier_size_);
        	}
	}
}
// only frontier centroids
void FrontierDetector::publishFrontierCentroids(const std::vector<Frontier>& frontiers) {
    geometry_msgs::PoseArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    
    for (const auto& frontier : frontiers) {
        geometry_msgs::Pose pose;
        pose.position.x = frontier.x;
        pose.position.y = frontier.y;
        pose.position.z = 0.0;
        pose.orientation.w = 1.0;
        msg.poses.push_back(pose);
    }
    
    frontier_pub_.publish(msg);
}

// all frontier points
void FrontierDetector::publishFrontierPoints(const std::vector<std::pair<int, int>>& points) {
    geometry_msgs::PoseArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    
    for (const auto& point : points) {
        double wx, wy;
        mapToWorld(point.first, point.second, wx, wy);
        
        geometry_msgs::Pose pose;
        pose.position.x = wx;
        pose.position.y = wy;
        pose.position.z = 0.0;
        pose.orientation.w = 1.0;
        msg.poses.push_back(pose);
    }
    
    frontier_points_pub_.publish(msg);
}

// rviz markers
void FrontierDetector::publishVisualizationMarkers(const std::vector<Frontier>& frontiers,
                                                  const std::vector<std::pair<int, int>>& points) {
    visualization_msgs::MarkerArray marker_array;
    
    // Points marker
    visualization_msgs::Marker points_marker;
    points_marker.header.frame_id = "map";
    points_marker.header.stamp = ros::Time::now();
    points_marker.ns = "frontier_points";
    points_marker.id = 0;
    points_marker.type = visualization_msgs::Marker::POINTS;
    points_marker.action = visualization_msgs::Marker::ADD;
    points_marker.scale.x = 0.05;
    points_marker.scale.y = 0.05;
    points_marker.color.r = 1.0;
    points_marker.color.g = 0.0;
    points_marker.color.b = 0.0;
    points_marker.color.a = 1.0;
    
    for (const auto& point : points) {
        double wx, wy;
        mapToWorld(point.first, point.second, wx, wy);
        
        geometry_msgs::Point p;
        p.x = wx;
        p.y = wy;
        p.z = 0.01;
        points_marker.points.push_back(p);
    }
    
    marker_array.markers.push_back(points_marker);
    
    // Centroid markers
    for (size_t i = 0; i < frontiers.size(); ++i) {
        visualization_msgs::Marker centroid_marker;
        centroid_marker.header.frame_id = "map";
        centroid_marker.header.stamp = ros::Time::now();
        centroid_marker.ns = "frontier_centroids";
        centroid_marker.id = i;
        centroid_marker.type = visualization_msgs::Marker::SPHERE;
        centroid_marker.action = visualization_msgs::Marker::ADD;
        centroid_marker.pose.position.x = frontiers[i].x;
        centroid_marker.pose.position.y = frontiers[i].y;
        centroid_marker.pose.position.z = 0.1;
        centroid_marker.pose.orientation.w = 1.0;
        centroid_marker.scale.x = 0.3;
        centroid_marker.scale.y = 0.3;
        centroid_marker.scale.z = 0.3;
        centroid_marker.color.r = 0.0;
        centroid_marker.color.g = 1.0;
        centroid_marker.color.b = 0.0;
        centroid_marker.color.a = 0.8;
        
        marker_array.markers.push_back(centroid_marker);
    }
    
    frontier_viz_pub_.publish(marker_array);
}

// Converts grid coordinates to world coords
void FrontierDetector::mapToWorld(int mx, int my, double& wx, double& wy) {
	if (!map_) return;
	wx = map_->info.origin.position.x + (mx + 0.5) * map_->info.resolution;
	wy = map_->info.origin.position.y + (my + 0.5) * map_->info.resolution;
}

// Converts world coords to grid coords
bool FrontierDetector::worldToMap(double wx, double wy, int& mx, int& my) {
	if (!map_) return false;

	mx = static_cast<int>((wx - map_->info.origin.position.x) / map_->info.resolution);
	my = static_cast<int>((wy - map_->info.origin.position.y) / map_->info.resolution);
	return (mx >= 0 && mx < static_cast<int>(map_->info.width) &&
            my >= 0 && my < static_cast<int>(map_->info.height));
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "frontier_detector");
    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    FrontierDetector detector(nh, private_nh);
    ros::spin();
    
    
    return 0;
}



