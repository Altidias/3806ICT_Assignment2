#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include "turtlebot3_explorer/CheckSurvivor.h"
#include "turtlebot3_explorer/GetSurvivors.h"
#include <vector>
#include <mutex>
#include "math_utils.hpp"

using namespace explorer_utils;

struct Survivor {
    int id;
    double x, y;
    bool found;
    std::string found_by;
    ros::Time found_time;
    
    Survivor(int id_, double x_, double y_) 
        : id(id_), x(x_), y(y_), found(false) {}
};

class SurvivorMgr {
public:
    SurvivorMgr(ros::NodeHandle& nh) : nh_(nh), all_found_(false) {
        // hardcoded locations
        survs_.emplace_back(0, -1.0, -2.0);   // living room
        survs_.emplace_back(1, 2.0, -1.0);   // kitchen
        
        // params
        nh_.param("discovery_radius", disc_r_, 0.5);
        nh_.param("robot_names", robots_, std::vector<std::string>{"robot1", "robot2"});
        
        // services
        check_srv_ = nh_.advertiseService("check_survivor", &SurvivorMgr::check, this);
        get_srv_ = nh_.advertiseService("get_survivors", &SurvivorMgr::get, this);
        
        // pubs
        all_pub_ = nh_.advertise<std_msgs::Bool>("/all_survivors_found", 1, true);
        status_pub_ = nh_.advertise<std_msgs::String>("/survivor_status", 1, true);
        viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/survivor_markers", 1, true);
        
        // robot subs
        for (const auto& n : robots_) {
            auto sub = nh_.subscribe<nav_msgs::Odometry>(
                "/" + n + "/odom", 1,
                [this, n](const nav_msgs::Odometry::ConstPtr& msg) {
                    this->check_pos(msg, n);
                }
            );
            subs_.push_back(sub);
        }
        
        // viz timer
        viz_timer_ = nh_.createTimer(ros::Duration(1.0), &SurvivorMgr::pub_viz, this);
        
        pub_status();
        
        // wait for service
        ros::Duration(0.5).sleep();
    }

private:
    ros::NodeHandle nh_;
    std::vector<Survivor> survs_;
    std::vector<std::string> robots_;
    double disc_r_;
    bool all_found_;
    std::mutex mtx_;
    
    // ros
    ros::ServiceServer check_srv_;
    ros::ServiceServer get_srv_;
    ros::Publisher all_pub_;
    ros::Publisher status_pub_;
    ros::Publisher viz_pub_;
    std::vector<ros::Subscriber> subs_;
    ros::Timer viz_timer_;
    
    void check_pos(const nav_msgs::Odometry::ConstPtr& msg, const std::string& robot) {
        std::lock_guard<std::mutex> lock(mtx_);
        
        double rx = msg->pose.pose.position.x;
        double ry = msg->pose.pose.position.y;
        
        // check survivors
        bool new_found = false;
        for (auto& s : survs_) {
            if (!s.found) {
                double d = euclideanDistance(rx, ry, s.x, s.y);
                
                if (d < disc_r_) {
                    s.found = true;
                    s.found_by = robot;
                    s.found_time = ros::Time::now();
                    new_found = true;
                }
            }
        }
        
        if (new_found) {
            check_all();
            pub_status();
        }
    }
    
    bool check(turtlebot3_explorer::CheckSurvivor::Request& req,
               turtlebot3_explorer::CheckSurvivor::Response& res) {
        std::lock_guard<std::mutex> lock(mtx_);
        
        // check if near position
        for (const auto& s : survs_) {
            double d = euclideanDistance(req.x, req.y, s.x, s.y);
            
            if (d < disc_r_) {
                res.found = true;
                res.survivor_id = s.id;
                res.already_discovered = s.found;
                return true;
            }
        }
        
        res.found = false;
        return true;
    }
    
    bool get(turtlebot3_explorer::GetSurvivors::Request& req,
             turtlebot3_explorer::GetSurvivors::Response& res) {
        std::lock_guard<std::mutex> lock(mtx_);
        
        res.all_found = all_found_;
        
        for (const auto& s : survs_) {
            res.survivor_ids.push_back(s.id);
            res.x_positions.push_back(s.x);
            res.y_positions.push_back(s.y);
            res.discovered.push_back(s.found);
            
            if (req.only_discovered && !s.found) {
                // remove last
                res.survivor_ids.pop_back();
                res.x_positions.pop_back();
                res.y_positions.pop_back();
                res.discovered.pop_back();
            }
        }
        
        return true;
    }
    
    void check_all() {
        bool all = true;
        for (const auto& s : survs_) {
            if (!s.found) {
                all = false;
                break;
            }
        }
        
        if (all && !all_found_) {
            all_found_ = true;
            
            std_msgs::Bool msg;
            msg.data = true;
            all_pub_.publish(msg);
        }
    }
    
    void pub_status() {
        int cnt = 0;
        for (const auto& s : survs_) {
            if (s.found) cnt++;
        }
        
        std_msgs::String msg;
        msg.data = "survivors: " + std::to_string(cnt) + "/" + 
                   std::to_string(survs_.size());
        
        if (all_found_) {
            msg.data += " - all found";
        }
        
        status_pub_.publish(msg);
    }
    
    void pub_viz(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(mtx_);
        
        visualization_msgs::MarkerArray markers;
        
        for (const auto& s : survs_) {
            visualization_msgs::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = ros::Time::now();
            m.ns = "survivors";
            m.id = s.id;
            m.type = visualization_msgs::Marker::CYLINDER;
            m.action = visualization_msgs::Marker::ADD;
            
            m.pose.position.x = s.x;
            m.pose.position.y = s.y;
            m.pose.position.z = 0.5;
            m.pose.orientation.w = 1.0;
            
            m.scale.x = 0.4;
            m.scale.y = 0.4;
            m.scale.z = 1.0;
            
            if (s.found) {
                // green
                m.color.r = 0.0;
                m.color.g = 1.0;
                m.color.b = 0.0;
                m.color.a = 0.9;
            } else {
                // red
                m.color.r = 1.0;
                m.color.g = 0.0;
                m.color.b = 0.0;
                m.color.a = 0.5;
            }
            
            markers.markers.push_back(m);
            
            // text label
            visualization_msgs::Marker txt;
            txt = m;
            txt.id = s.id + 100;
            txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            txt.pose.position.z = 1.2;
            txt.scale.z = 0.3;
            txt.color.r = 1.0;
            txt.color.g = 1.0;
            txt.color.b = 1.0;
            txt.color.a = 1.0;
            
            if (s.found) {
                txt.text = "S" + std::to_string(s.id) + " FOUND";
            } else {
                txt.text = "S" + std::to_string(s.id);
            }
            
            markers.markers.push_back(txt);
        }
        
        viz_pub_.publish(markers);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "survivor_manager");
    ros::NodeHandle nh("~");
    
    SurvivorMgr mgr(nh);
    ros::spin();
    
    return 0;
}