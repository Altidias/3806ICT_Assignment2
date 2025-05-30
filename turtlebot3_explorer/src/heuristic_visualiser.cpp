#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <mutex>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "math_utils.hpp"

using namespace explorer_utils;

class Visualizer {
public:
    Visualizer(ros::NodeHandle& nh) : nh_(nh) {
        // params
        nh_.param("robot_names", robots_, std::vector<std::string>{"robot1", "robot2"});
        nh_.param("visualization_rate", viz_rate_, 2.0);
        nh_.param("heatmap_alpha", heatmap_alpha_, 0.6);
        nh_.param("export_data", export_data_, true);
        nh_.param("export_path", export_path_, std::string("/tmp/exploration_data"));
        
        // publishers
        heat_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/heuristic_heatmap", 1, true);
        overlap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/overlap_heatmap", 1, true);
        frontier_attr_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/frontier_attraction_map", 1, true);
        combined_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/combined_heuristic_map", 1, true);
        heat_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/heat_markers", 1, true);
        raw_data_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/heuristic_raw_data", 1, true);
        
        // subscribers
        map_sub_ = nh_.subscribe("/map", 1, &Visualizer::map_cb, this);
        frontiers_sub_ = nh_.subscribe("/frontiers", 1, &Visualizer::frontiers_cb, this);
        
        // robot odom subs
        for (const auto& robot : robots_) {
            auto sub = nh_.subscribe<nav_msgs::Odometry>(
                "/" + robot + "/odom", 1,
                [this, robot](const nav_msgs::Odometry::ConstPtr& msg) {
                    this->odom_cb(msg, robot);
                }
            );
            odom_subs_.push_back(sub);
        }
        
        viz_timer_ = nh_.createTimer(ros::Duration(1.0/viz_rate_), &Visualizer::viz_cb, this);
        
        if (export_data_) {
            init_export();
        }
        
        ROS_INFO("Visualizer ready");
    }

private:
    ros::NodeHandle nh_;
    std::vector<std::string> robots_;
    double viz_rate_;
    double heatmap_alpha_;
    bool export_data_;
    std::string export_path_;
    
    // ros stuff
    std::vector<ros::Subscriber> odom_subs_;
    ros::Subscriber map_sub_;
    ros::Subscriber frontiers_sub_;
    ros::Publisher heat_pub_;
    ros::Publisher overlap_pub_;
    ros::Publisher frontier_attr_pub_;
    ros::Publisher combined_pub_;
    ros::Publisher heat_markers_pub_;
    ros::Publisher raw_data_pub_;
    ros::Timer viz_timer_;
    
    // state
    std::mutex mtx_;
    nav_msgs::OccupancyGrid::ConstPtr map_;
    std::map<std::string, geometry_msgs::Pose> robot_poses_;
    std::map<std::string, std::vector<geometry_msgs::Pose>> robot_paths_;
    std::vector<std::pair<double, double>> frontiers_;
    
    // data export
    std::ofstream data_file_;
    int frame_counter_ = 0;
    
    void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        map_ = msg;
    }
    
    void frontiers_cb(const geometry_msgs::PoseArray::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        frontiers_.clear();
        for (const auto& pose : msg->poses) {
            frontiers_.emplace_back(pose.position.x, pose.position.y);
        }
    }
    
    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg, const std::string& robot) {
        std::lock_guard<std::mutex> lock(mtx_);
        robot_poses_[robot] = msg->pose.pose;
        
        // store path history
        robot_paths_[robot].push_back(msg->pose.pose);
        if (robot_paths_[robot].size() > 100) {  // keep last 100
            robot_paths_[robot].erase(robot_paths_[robot].begin());
        }
    }
    
    void viz_cb(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(mtx_);
        
        if (!map_) {
            return;
        }
        
        // generate maps
        auto heat_map = gen_heat();
        auto overlap_map = gen_overlap();
        auto frontier_map = gen_frontier();
        auto combined_map = gen_combined();
        
        // publish everything
        pub_grid(heat_map, heat_pub_, "heat");
        pub_grid(overlap_map, overlap_pub_, "overlap");
        pub_grid(frontier_map, frontier_attr_pub_, "frontier_attraction");
        pub_grid(combined_map, combined_pub_, "combined");
        
        pub_3d(heat_map, heat_markers_pub_, "heat");
        pub_raw(heat_map, overlap_map, frontier_map, combined_map);
        
        if (export_data_) {
            export_frame(heat_map, overlap_map, frontier_map, combined_map);
        }
        
        frame_counter_++;
    }
    
    std::vector<std::vector<double>> gen_heat() {
        int w = map_->info.width;
        int h = map_->info.height;
        std::vector<std::vector<double>> heat(h, std::vector<double>(w, 0.0));
        
        // generate heat based on robot distance and paths
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                double wx, wy;
                mapToWorld(map_, x, y, wx, wy);
                
                double heat_value = 0.0;
                
                // heat from current robot positions
                for (const auto& [robot_id, pose] : robot_poses_) {
                    double dist = euclideanDistance(wx, wy, pose.position.x, pose.position.y);
                    if (dist < 3.0) {  // within 3m
                        heat_value += (3.0 - dist) / 3.0 * 0.5;  // closer = more heat
                    }
                }
                
                // heat from robot paths
                for (const auto& [robot_id, path] : robot_paths_) {
                    for (const auto& path_pose : path) {
                        double dist = euclideanDistance(wx, wy, path_pose.position.x, path_pose.position.y);
                        if (dist < 1.0) {  // within 1m of path
                            heat_value += 0.1;  // visited areas
                        }
                    }
                }
                
                heat[y][x] = std::min(heat_value, 2.0);  // cap at 2.0
            }
        }
        
        return heat;
    }
    
    std::vector<std::vector<double>> gen_overlap() {
        int w = map_->info.width;
        int h = map_->info.height;
        std::vector<std::vector<double>> overlap(h, std::vector<double>(w, 0.0));
        
        // calc robot proximity overlap
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                double wx, wy;
                mapToWorld(map_, x, y, wx, wy);
                
                int nearby_robots = 0;
                for (const auto& [robot_id, pose] : robot_poses_) {
                    double dist = euclideanDistance(wx, wy, pose.position.x, pose.position.y);
                    if (dist < 2.0) { // within 2m
                        nearby_robots++;
                    }
                }
                
                if (nearby_robots > 1) {
                    overlap[y][x] = (nearby_robots - 1) * 0.5;
                }
            }
        }
        
        return overlap;
    }
    
    std::vector<std::vector<double>> gen_frontier() {
        int w = map_->info.width;
        int h = map_->info.height;
        std::vector<std::vector<double>> attraction(h, std::vector<double>(w, 0.0));
        
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                double wx, wy;
                mapToWorld(map_, x, y, wx, wy);
                
                double max_attraction = 0.0;
                for (const auto& [fx, fy] : frontiers_) {
                    double dist = euclideanDistance(wx, wy, fx, fy);
                    if (dist < 3.0) { // within 3m of frontier
                        double attr = (3.0 - dist) / 3.0;
                        max_attraction = std::max(max_attraction, attr);
                    }
                }
                
                attraction[y][x] = max_attraction;
            }
        }
        
        return attraction;
    }
    
    std::vector<std::vector<double>> gen_combined() {
        auto heat = gen_heat();
        auto overlap = gen_overlap();
        auto frontier = gen_frontier();
        
        int h = heat.size();
        int w = heat[0].size();
        std::vector<std::vector<double>> combined(h, std::vector<double>(w, 0.0));
        
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                combined[y][x] = heat[y][x] + overlap[y][x] + frontier[y][x];
            }
        }
        
        return combined;
    }
    
    void pub_grid(const std::vector<std::vector<double>>& data, 
                  ros::Publisher& pub, const std::string& type) {
        nav_msgs::OccupancyGrid grid;
        grid.header.stamp = ros::Time::now();
        grid.header.frame_id = "map";
        grid.info = map_->info;
        
        int h = data.size();
        int w = data[0].size();
        grid.data.resize(w * h);
        
        // find min/max for normalization
        double min_val = 0.0, max_val = 0.0;
        for (const auto& row : data) {
            for (double val : row) {
                min_val = std::min(min_val, val);
                max_val = std::max(max_val, val);
            }
        }
        
        // normalize to 0-100
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                double normalized = 0.0;
                if (max_val > min_val) {
                    normalized = (data[y][x] - min_val) / (max_val - min_val);
                }
                grid.data[y * w + x] = static_cast<int8_t>(normalized * 100);
            }
        }
        
        pub.publish(grid);
    }
    
    void pub_3d(const std::vector<std::vector<double>>& data,
                ros::Publisher& pub, const std::string& type) {
        visualization_msgs::MarkerArray markers;
        
        // clear previous
        visualization_msgs::Marker clear_marker;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(clear_marker);
        
        int h = data.size();
        int w = data[0].size();
        
        // find max for height scaling
        double max_val = 0.0;
        for (const auto& row : data) {
            for (double val : row) {
                max_val = std::max(max_val, val);
            }
        }
        
        if (max_val <= 0.0) return;
        
        int marker_id = 0;
        for (int y = 0; y < h; y += 4) { // sample every 4 pixels
            for (int x = 0; x < w; x += 4) {
                if (data[y][x] <= 0.0) continue;
                
                visualization_msgs::Marker marker;
                marker.header.stamp = ros::Time::now();
                marker.header.frame_id = "map";
                marker.ns = type + "_3d";
                marker.id = marker_id++;
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;
                
                double wx, wy;
                mapToWorld(map_, x, y, wx, wy);
                
                marker.pose.position.x = wx;
                marker.pose.position.y = wy;
                marker.pose.position.z = (data[y][x] / max_val) * 0.5;
                marker.pose.orientation.w = 1.0;
                
                marker.scale.x = 0.2;
                marker.scale.y = 0.2;
                marker.scale.z = (data[y][x] / max_val) * 1.0;
                
                marker.color.r = data[y][x] / max_val;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = heatmap_alpha_;
                
                markers.markers.push_back(marker);
            }
        }
        
        pub.publish(markers);
    }
    
    void pub_raw(const std::vector<std::vector<double>>& heat,
                 const std::vector<std::vector<double>>& overlap,
                 const std::vector<std::vector<double>>& frontier,
                 const std::vector<std::vector<double>>& combined) {
        std_msgs::Float32MultiArray msg;
        
        int h = heat.size();
        int w = heat[0].size();
        
        msg.layout.dim.resize(3);
        msg.layout.dim[0].label = "type";
        msg.layout.dim[0].size = 4;
        msg.layout.dim[1].label = "height";
        msg.layout.dim[1].size = h;
        msg.layout.dim[2].label = "width";
        msg.layout.dim[2].size = w;
        
        msg.data.resize(4 * h * w);
        
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                int base_idx = y * w + x;
                msg.data[0 * h * w + base_idx] = heat[y][x];
                msg.data[1 * h * w + base_idx] = overlap[y][x];
                msg.data[2 * h * w + base_idx] = frontier[y][x];
                msg.data[3 * h * w + base_idx] = combined[y][x];
            }
        }
        
        raw_data_pub_.publish(msg);
    }
    
    void init_export() {
        std::string filename = export_path_ + "/heuristic_data_" + 
                              std::to_string(ros::Time::now().sec) + ".csv";
        data_file_.open(filename);
        
        if (data_file_.is_open()) {
            data_file_ << "frame,timestamp,robot_count,frontier_count,avg_heat,max_heat,avg_overlap,max_overlap\n";
        }
    }
    
    void export_frame(const std::vector<std::vector<double>>& heat,
                      const std::vector<std::vector<double>>& overlap,
                      const std::vector<std::vector<double>>& frontier,
                      const std::vector<std::vector<double>>& combined) {
        if (!data_file_.is_open()) return;
        
        // calc stats
        double avg_heat = 0.0, max_heat = 0.0;
        double avg_overlap = 0.0, max_overlap = 0.0;
        int cell_count = 0;
        
        int h = heat.size();
        int w = heat[0].size();
        
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                avg_heat += heat[y][x];
                max_heat = std::max(max_heat, heat[y][x]);
                avg_overlap += overlap[y][x];
                max_overlap = std::max(max_overlap, overlap[y][x]);
                cell_count++;
            }
        }
        
        if (cell_count > 0) {
            avg_heat /= cell_count;
            avg_overlap /= cell_count;
        }
        
        data_file_ << frame_counter_ << ","
                  << ros::Time::now().toSec() << ","
                  << robot_poses_.size() << ","
                  << frontiers_.size() << ","
                  << std::fixed << std::setprecision(4) << avg_heat << ","
                  << max_heat << ","
                  << avg_overlap << ","
                  << max_overlap << "\n";
        data_file_.flush();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "heuristic_visualizer");
    ros::NodeHandle nh("~");
    
    Visualizer visualizer(nh);
    
    ros::spin();
    return 0;
}