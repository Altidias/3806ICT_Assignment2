#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <vector>
#include <queue>
#include <map>
#include <set>
#include "math_utils.hpp"

using namespace explorer_utils;

class FrontierDetector {
public:
    FrontierDetector(ros::NodeHandle& nh) : nh_(nh) {
        // params
        nh_.param("map_topic", map_topic_, std::string("/map"));
        nh_.param("frontier_topic", front_topic_, std::string("/frontiers"));
        nh_.param("min_frontier_size", min_size_, 8);
        nh_.param("update_rate", rate_, 2.0);
        nh_.param("robot_filter_radius", r_filter_, 0.8);
        nh_.param("frontier_spacing", spacing_, 1.0);
        nh_.param("min_distance_from_obstacles", min_obs_dist_, 0.5);
        nh_.param("reachability_check", enable_reach_check_, true);
        
        nh_.param("robot_names", robots_, std::vector<std::string>{"robot1", "robot2"});
        
        // pub/sub
        map_sub_ = nh_.subscribe(map_topic_, 1, &FrontierDetector::map_cb, this);
        front_pub_ = nh_.advertise<geometry_msgs::PoseArray>(front_topic_, 1, true);
        
        // robot subs
        for (const auto& name : robots_) {
            auto sub = nh_.subscribe<nav_msgs::Odometry>(
                "/" + name + "/odom", 1,
                [this, name](const nav_msgs::Odometry::ConstPtr& msg) {
                    this->odom_cb(msg, name);
                }
            );
            odom_subs_.push_back(sub);
        }
        
        timer_ = nh_.createTimer(ros::Duration(1.0/rate_), 
                                &FrontierDetector::detect_cb, this);
        
        ROS_INFO("Frontier detector ready");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Publisher front_pub_;
    ros::Timer timer_;
    std::vector<ros::Subscriber> odom_subs_;
    
    std::string map_topic_;
    std::string front_topic_;
    int min_size_;
    double rate_;
    double r_filter_;
    double spacing_;
    double min_obs_dist_;
    bool enable_reach_check_;
    std::vector<std::string> robots_;
    
    nav_msgs::OccupancyGrid::ConstPtr map_;
    std::map<std::string, geometry_msgs::Pose> poses_;

    void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& map) {
        map_ = map;
    }
    
    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg, const std::string& name) {
        poses_[name] = msg->pose.pose;
    }

    void detect_cb(const ros::TimerEvent&) {
        if (!map_) {
            return;
        }
        
        auto fronts = detect();
        publish(fronts);
    }

    std::vector<std::pair<double, double>> detect() {
        std::vector<std::pair<int, int>> cells;
        
        int w = map_->info.width;
        int h = map_->info.height;
        
        // find frontier cells
        for (int y = 1; y < h - 1; ++y) {
            for (int x = 1; x < w - 1; ++x) {
                if (is_frontier(x, y)) {
                    cells.emplace_back(x, y);
                }
            }
        }
        
        // cluster and extract
        auto clusters = cluster(cells);
        auto fronts = extract(clusters);
        
        return filter_reachable(fronts);
    }

    bool is_frontier(int x, int y) {
        int idx = y * map_->info.width + x;
        
        // must be free
        if (map_->data[idx] != 0) return false;
        
        // check distance from obstacles
        if (!is_safe_distance(x, y)) {
            return false;
        }
        
        // check 8-neighbors for unknown
        const int w = map_->info.width;
        const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        
        int unknown_neighbors = 0;
        for (int i = 0; i < 8; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            
            if (nx >= 0 && nx < w && ny >= 0 && ny < map_->info.height) {
                int nidx = ny * w + nx;
                if (map_->data[nidx] == -1) {
                    unknown_neighbors++;
                }
            }
        }
        
        // need multiple unknown neighbors
        return unknown_neighbors >= 2;
    }

    bool is_safe_distance(int x, int y) {
        int safety_radius = (int)std::ceil(min_obs_dist_ / map_->info.resolution);
        
        for (int dy = -safety_radius; dy <= safety_radius; ++dy) {
            for (int dx = -safety_radius; dx <= safety_radius; ++dx) {
                int nx = x + dx;
                int ny = y + dy;
                
                if (nx >= 0 && nx < map_->info.width && 
                    ny >= 0 && ny < map_->info.height) {
                    
                    int idx = ny * map_->info.width + nx;
                    if (map_->data[idx] > 50) {  // obstacle
                        double dist = std::sqrt(dx*dx + dy*dy) * map_->info.resolution;
                        if (dist < min_obs_dist_) {
                            return false;
                        }
                    }
                }
            }
        }
        
        return true;
    }

    std::vector<std::vector<std::pair<int, int>>> cluster(
        const std::vector<std::pair<int, int>>& cells) {
        
        std::vector<std::vector<std::pair<int, int>>> clusters;
        std::vector<bool> vis(cells.size(), false);
        
        // bfs cluster
        for (size_t i = 0; i < cells.size(); ++i) {
            if (vis[i]) continue;
            
            std::vector<std::pair<int, int>> c;
            std::queue<size_t> q;
            
            q.push(i);
            vis[i] = true;
            
            while (!q.empty()) {
                size_t cur = q.front();
                q.pop();
                c.push_back(cells[cur]);
                
                // check adjacency
                for (size_t j = 0; j < cells.size(); ++j) {
                    if (vis[j]) continue;
                    
                    int dx = std::abs(cells[j].first - cells[cur].first);
                    int dy = std::abs(cells[j].second - cells[cur].second);
                    
                    if (dx <= 3 && dy <= 3) {
                        q.push(j);
                        vis[j] = true;
                    }
                }
            }
            
            // keep reasonably sized clusters
            if ((int)c.size() >= min_size_) {
                clusters.push_back(c);
            }
        }
        
        return clusters;
    }

    std::vector<std::pair<double, double>> extract(
        const std::vector<std::vector<std::pair<int, int>>>& clusters) {
        
        std::vector<std::pair<double, double>> fronts;
        
        for (const auto& c : clusters) {
            if (c.size() < 20) {
                // small cluster - use centroid
                double sx = 0, sy = 0;
                for (const auto& cell : c) {
                    double wx, wy;
                    mapToWorld(map_, cell.first, cell.second, wx, wy);
                    sx += wx;
                    sy += wy;
                }
                
                fronts.emplace_back(sx / c.size(), sy / c.size());
            } else {
                // larger cluster - sample multiple points
                std::vector<std::pair<double, double>> pts;
                
                // convert to world
                for (const auto& cell : c) {
                    double wx, wy;
                    mapToWorld(map_, cell.first, cell.second, wx, wy);
                    pts.emplace_back(wx, wy);
                }
                
                // sample with spacing
                std::vector<std::pair<double, double>> samp;
                
                for (const auto& p : pts) {
                    bool close = false;
                    for (const auto& e : samp) {
                        double d = euclideanDistance(
                            p.first, p.second,
                            e.first, e.second
                        );
                        if (d < spacing_) {
                            close = true;
                            break;
                        }
                    }
                    
                    if (!close) {
                        samp.push_back(p);
                    }
                }
                
                fronts.insert(fronts.end(), samp.begin(), samp.end());
            }
        }
        
        return fronts;
    }

    std::vector<std::pair<double, double>> filter_reachable(
        const std::vector<std::pair<double, double>>& fronts) {
        
        std::vector<std::pair<double, double>> filtered;
        
        for (const auto& f : fronts) {
            bool valid = true;
            
            // filter if too close to robot
            for (const auto& [name, pose] : poses_) {
                double d = euclideanDistance(
                    f.first, f.second,
                    pose.position.x, pose.position.y
                );
                
                if (d < r_filter_) {
                    valid = false;
                    break;
                }
            }
            
            if (!valid) continue;
            
            // check reachability
            if (enable_reach_check_ && !is_reachable(f)) {
                continue;
            }
            
            filtered.push_back(f);
        }
        
        return filtered;
    }

    bool is_reachable(const std::pair<double, double>& frontier) {
        // simple reachability check using flood fill
        if (poses_.empty()) return true;
        
        // find nearest robot
        double min_dist = std::numeric_limits<double>::max();
        geometry_msgs::Pose nearest_robot;
        
        for (const auto& [name, pose] : poses_) {
            double d = euclideanDistance(
                frontier.first, frontier.second,
                pose.position.x, pose.position.y
            );
            if (d < min_dist) {
                min_dist = d;
                nearest_robot = pose;
            }
        }
        
        // convert to map coords
        int robot_mx, robot_my, frontier_mx, frontier_my;
        
        if (!worldToMap(map_, nearest_robot.position.x, nearest_robot.position.y, robot_mx, robot_my) ||
            !worldToMap(map_, frontier.first, frontier.second, frontier_mx, frontier_my)) {
            return false;  // out of bounds
        }
        
        return connected(robot_mx, robot_my, frontier_mx, frontier_my);
    }

    bool connected(int start_x, int start_y, int goal_x, int goal_y) {
        int w = map_->info.width;
        int h = map_->info.height;
        
        if (start_x < 0 || start_x >= w || start_y < 0 || start_y >= h ||
            goal_x < 0 || goal_x >= w || goal_y < 0 || goal_y >= h) {
            return false;
        }
        
        std::vector<std::vector<bool>> visited(h, std::vector<bool>(w, false));
        std::queue<std::pair<int, int>> q;
        
        q.push({start_x, start_y});
        visited[start_y][start_x] = true;
        
        const int dx[] = {-1, 0, 1, 0};
        const int dy[] = {0, -1, 0, 1};
        
        int iterations = 0;
        const int max_iterations = 1000;  // prevent hanging
        
        while (!q.empty() && iterations < max_iterations) {
            iterations++;
            
            auto [x, y] = q.front();
            q.pop();
            
            if (x == goal_x && y == goal_y) {
                return true;  // found path
            }
            
            for (int i = 0; i < 4; ++i) {
                int nx = x + dx[i];
                int ny = y + dy[i];
                
                if (nx >= 0 && nx < w && ny >= 0 && ny < h && !visited[ny][nx]) {
                    int idx = ny * w + nx;
                    
                    // allow free space and unknown
                    if (map_->data[idx] == 0 || map_->data[idx] == -1) {
                        visited[ny][nx] = true;
                        q.push({nx, ny});
                    }
                }
            }
        }
        
        return false;  // no path
    }

    void publish(const std::vector<std::pair<double, double>>& fronts) {
        geometry_msgs::PoseArray msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        
        for (const auto& f : fronts) {
            geometry_msgs::Pose p;
            p.position.x = f.first;
            p.position.y = f.second;
            p.position.z = 0.0;
            p.orientation.w = 1.0;
            msg.poses.push_back(p);
        }
        
        front_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "frontier_detector");
    ros::NodeHandle nh("~");
    
    FrontierDetector detector(nh);
    ros::spin();
    
    return 0;
}