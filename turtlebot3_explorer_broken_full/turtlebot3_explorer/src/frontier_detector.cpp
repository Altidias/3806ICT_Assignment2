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
        nh_.param("min_frontier_size", min_size_, 12);
        nh_.param("update_rate", rate_, 2.0);
        nh_.param("robot_filter_radius", r_filter_, 0.3);
        nh_.param("frontier_spacing", spacing_, 0.5);
        
        // robot names
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
        
        // timer
        timer_ = nh_.createTimer(ros::Duration(1.0/rate_), 
                                &FrontierDetector::detect_cb, this);
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
        
        // cluster
        auto clusters = cluster(cells);
        
        // extract points
        auto fronts = extract(clusters);
        
        // filter
        return filter(fronts);
    }

    bool is_frontier(int x, int y) {
        int idx = y * map_->info.width + x;
        
        // must be free
        if (map_->data[idx] != 0) return false;
        
        // check 8-neighbors for unknown
        const int w = map_->info.width;
        const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        
        for (int i = 0; i < 8; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            
            if (nx >= 0 && nx < w && ny >= 0 && ny < map_->info.height) {
                int nidx = ny * w + nx;
                if (map_->data[nidx] == -1) {
                    return true;
                }
            }
        }
        
        return false;
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
                    
                    // 8-connected with gaps
                    int dx = std::abs(cells[j].first - cells[cur].first);
                    int dy = std::abs(cells[j].second - cells[cur].second);
                    
                    if (dx <= 2 && dy <= 2) {
                        q.push(j);
                        vis[j] = true;
                    }
                }
            }
            
            // keep small clusters
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
            // small cluster - use centroid
            if (c.size() < 10) {
                double sx = 0, sy = 0;
                for (const auto& cell : c) {
                    double wx, wy;
                    mapToWorld(map_, cell.first, cell.second, wx, wy);
                    sx += wx;
                    sy += wy;
                }
                
                fronts.emplace_back(sx / c.size(), sy / c.size());
            } else {
                // larger - sample points
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
                
                // add sampled
                fronts.insert(fronts.end(), samp.begin(), samp.end());
            }
        }
        
        return fronts;
    }

    std::vector<std::pair<double, double>> filter(
        const std::vector<std::pair<double, double>>& fronts) {
        
        std::vector<std::pair<double, double>> filt;
        
        for (const auto& f : fronts) {
            bool close = false;
            
            // filter if very close to robot
            for (const auto& [name, pose] : poses_) {
                double d = euclideanDistance(
                    f.first, f.second,
                    pose.position.x, pose.position.y
                );
                
                if (d < r_filter_) {
                    close = true;
                    break;
                }
            }
            
            if (!close) {
                filt.push_back(f);
            }
        }
        
        return filt;
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