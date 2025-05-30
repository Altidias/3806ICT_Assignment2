#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include "turtlebot3_explorer/GetPathCosts.h"
#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <numeric>
#include "math_utils.hpp"

using namespace explorer_utils;

struct RobotInfo {
    std::string id;
    geometry_msgs::Pose pose;
    std::vector<std::pair<int, int>> path;
    ros::Time last_update;
    
    RobotInfo() : last_update(ros::Time::now()) {}
};

class HeuristicCalc {
public:
    HeuristicCalc(ros::NodeHandle& nh) : nh_(nh) {
        // params
        nh_.param("robot_names", robots_, std::vector<std::string>{"robot1", "robot2"});
        nh_.param("overlap_radius", overlap_r_, 1.0);
        nh_.param("overlap_cost_factor", overlap_f_, 0.5);
        nh_.param("path_decay_time", decay_t_, 10.0);
        nh_.param("frontier_attraction_radius", front_r_, 2.0);
        nh_.param("frontier_cost_reduction", front_red_, 0.3);
        
        // service
        srv_ = nh_.advertiseService("get_path_costs", &HeuristicCalc::get_costs, this);
        
        // subs
        map_sub_ = nh_.subscribe("/map", 1, &HeuristicCalc::map_cb, this);
        front_sub_ = nh_.subscribe("/frontiers", 1, &HeuristicCalc::front_cb, this);
        
        // robot odom subs
        for (const auto& n : robots_) {
            rob_info_[n] = RobotInfo();
            rob_info_[n].id = n;
            
            auto sub = nh_.subscribe<nav_msgs::Odometry>(
                "/" + n + "/odom", 1,
                [this, n](const nav_msgs::Odometry::ConstPtr& msg) {
                    this->odom_cb(msg, n);
                }
            );
            odom_subs_.push_back(sub);
        }
        
        // init heat
        init_heat();
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceServer srv_;
    ros::Subscriber map_sub_;
    ros::Subscriber front_sub_;
    std::vector<ros::Subscriber> odom_subs_;
    
    // params
    std::vector<std::string> robots_;
    double overlap_r_;
    double overlap_f_;
    double decay_t_;
    double front_r_;
    double front_red_;
    
    // state
    std::mutex mtx_;
    nav_msgs::OccupancyGrid::ConstPtr map_;
    std::vector<std::pair<double, double>> fronts_;
    std::unordered_map<std::string, RobotInfo> rob_info_;
    std::vector<std::vector<double>> heat_;
    
    void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& map) {
        std::lock_guard<std::mutex> lock(mtx_);
        map_ = map;
        update_heat_size();
    }
    
    void front_cb(const geometry_msgs::PoseArray::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        fronts_.clear();
        for (const auto& p : msg->poses) {
            fronts_.emplace_back(p.position.x, p.position.y);
        }
    }
    
    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg, const std::string& n) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (rob_info_.find(n) != rob_info_.end()) {
            rob_info_[n].pose = msg->pose.pose;
            rob_info_[n].last_update = ros::Time::now();
        }
    }
    
    bool get_costs(turtlebot3_explorer::GetPathCosts::Request& req,
                   turtlebot3_explorer::GetPathCosts::Response& res) {
        std::lock_guard<std::mutex> lock(mtx_);
        
        if (!map_) {
            res.success = false;
            return true;
        }
        
        // calc costs
        res.costs.clear();
        res.costs.reserve(req.path_x.size());
        
        for (size_t i = 0; i < req.path_x.size(); ++i) {
            double c = calc_cost(req.robot_id, req.path_x[i], req.path_y[i]);
            res.costs.push_back(c);
        }
        
        // update path
        update_path(req.robot_id, req.path_x, req.path_y);
        
        res.success = true;
        
        return true;
    }
    
    double calc_cost(const std::string& rid, int mx, int my) {
        double cost = 1.0;
        
        // to world
        double wx, wy;
        mapToWorld(map_, mx, my, wx, wy);
        
        // overlap
        cost += calc_overlap(rid, wx, wy);
        
        // heat
        cost += calc_heat(mx, my);
        
        // frontier attraction
        cost -= calc_front(wx, wy);
        
        // proximity
        cost += calc_prox(rid, wx, wy);
        
        return std::max(0.1, cost);
    }
    
    double calc_overlap(const std::string& rid, double wx, double wy) {
        double overlap = 0.0;
        ros::Time now = ros::Time::now();
        
        for (const auto& [oid, info] : rob_info_) {
            if (oid == rid) continue;
            
            // skip old
            double age = (now - info.last_update).toSec();
            if (age > decay_t_) continue;
            
            // check path overlap
            for (const auto& [px, py] : info.path) {
                double pwx, pwy;
                mapToWorld(map_, px, py, pwx, pwy);
                
                double d = euclideanDistance(wx, wy, pwx, pwy);
                
                if (d < overlap_r_) {
                    double prox = 1.0 - (d / overlap_r_);
                    double time = 1.0 - (age / decay_t_);
                    overlap += overlap_f_ * prox * time;
                }
            }
        }
        
        return overlap;
    }
    
    double calc_heat(int mx, int my) {
        if (heat_.empty() || 
            my < 0 || my >= (int)heat_.size() ||
            mx < 0 || mx >= (int)heat_[0].size()) {
            return 0.0;
        }
        
        return heat_[my][mx] * 0.2;
    }
    
    double calc_front(double wx, double wy) {
        double attr = 0.0;
        
        for (const auto& [fx, fy] : fronts_) {
            double d = euclideanDistance(wx, wy, fx, fy);
            
            if (d < front_r_) {
                double p = 1.0 - (d / front_r_);
                attr = std::max(attr, front_red_ * p);
            }
        }
        
        return attr;
    }
    
    double calc_prox(const std::string& rid, double wx, double wy) {
        double prox = 0.0;
        
        for (const auto& [oid, info] : rob_info_) {
            if (oid == rid) continue;
            
            double d = euclideanDistance(wx, wy, 
                                       info.pose.position.x, 
                                       info.pose.position.y);
            
            if (d < 1.5) {
                prox += 0.3 * (1.5 - d) / 1.5;
            }
        }
        
        return prox;
    }
    
    void update_path(const std::string& rid, 
                     const std::vector<int32_t>& px,
                     const std::vector<int32_t>& py) {
        if (rob_info_.find(rid) != rob_info_.end()) {
            auto& info = rob_info_[rid];
            info.path.clear();
            info.path.reserve(px.size());
            
            for (size_t i = 0; i < px.size(); ++i) {
                info.path.emplace_back(px[i], py[i]);
            }
            
            info.last_update = ros::Time::now();
            
            // update heat
            update_heat(px, py);
        }
    }
    
    void init_heat() {
        heat_.clear();
    }
    
    void update_heat_size() {
        if (!map_) return;
        
        int w = map_->info.width;
        int h = map_->info.height;
        
        if (heat_.size() != h || 
            (heat_.size() > 0 && heat_[0].size() != w)) {
            heat_.assign(h, std::vector<double>(w, 0.0));
        }
    }
    
    void update_heat(const std::vector<int32_t>& px,
                     const std::vector<int32_t>& py) {
        if (heat_.empty()) return;
        
        double inc = 0.1;
        double decay = 0.99;
        
        // decay
        for (auto& row : heat_) {
            for (auto& c : row) {
                c *= decay;
            }
        }
        
        // add heat
        for (size_t i = 0; i < px.size(); ++i) {
            int x = px[i];
            int y = py[i];
            
            if (y >= 0 && y < (int)heat_.size() && 
                x >= 0 && x < (int)heat_[0].size()) {
                heat_[y][x] = std::min(5.0, heat_[y][x] + inc);
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "heuristic_calculator");
    ros::NodeHandle nh("~");
    
    HeuristicCalc calc(nh);
    
    ros::spin();
    
    return 0;
}