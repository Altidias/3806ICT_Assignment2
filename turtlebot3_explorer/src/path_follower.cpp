#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include "turtlebot3_explorer/FollowPath.h"
#include <tf2/utils.h>
#include <cmath>
#include <vector>
#include "math_utils.hpp"


using namespace explorer_utils;

class PathFollower {
public:
    PathFollower(ros::NodeHandle& nh, const std::string& name) 
        : nh_(nh), name_(name), idx_(0) {
        
        // params
        nh_.param("position_tolerance", pos_tol_, 0.25);
        nh_.param("linear_speed", max_lin_, 0.15);
        nh_.param("angular_speed", max_ang_, 0.5);
        nh_.param("goal_tolerance", goal_tol_, 0.3);
        nh_.param("lookahead_points", lookahead_, 0);
        
        // obstacle params
        nh_.param("critical_distance", crit_dist_, 0.1);
        nh_.param("slow_distance", slow_dist_, 0.3);
        
        // pubs/subs
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/" + name_ + "/cmd_vel", 1);
        path_viz_pub_ = nh_.advertise<nav_msgs::Path>("/" + name_ + "/planned_path", 1, true);  // ADD THIS
        done_pub_ = nh_.advertise<std_msgs::Bool>("/" + name_ + "/path_complete", 1);
        prog_pub_ = nh_.advertise<std_msgs::Int32>("/" + name_ + "/path_progress", 1);
        
        odom_sub_ = nh_.subscribe("/" + name_ + "/odom", 1, 
                                  &PathFollower::odom_cb, this);
        scan_sub_ = nh_.subscribe("/" + name_ + "/scan", 1,
                                  &PathFollower::scan_cb, this);
        
        // service
        srv_ = nh_.advertiseService("/" + name_ + "/follow_path", 
                                   &PathFollower::follow_srv, this);
        
        // control timer
        timer_ = nh_.createTimer(ros::Duration(0.1), &PathFollower::control_cb, this);
    }

private:
    ros::NodeHandle nh_;
    std::string name_;
    
    // ros stuff
    ros::Publisher cmd_pub_;
    ros::Publisher path_viz_pub_;
    ros::Publisher done_pub_;
    ros::Publisher prog_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;
    ros::ServiceServer srv_;
    ros::Timer timer_;
    
    // params
    double pos_tol_;
    double max_lin_;
    double max_ang_;
    double goal_tol_;
    int lookahead_;
    double crit_dist_;
    double slow_dist_;
    
    // state
    geometry_msgs::Pose pose_;
    std::vector<std::pair<double, double>> path_;
    size_t idx_;
    bool has_odom_ = false;
    bool has_scan_ = false;
    sensor_msgs::LaserScan scan_;
    
    // obstacle
    double min_dist_ = 10.0;
    
    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
        pose_ = msg->pose.pose;
        has_odom_ = true;
    }
    
    void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg) {
        scan_ = *msg;
        has_scan_ = true;
        update_dist();
    }
    
    void update_dist() {
        if (scan_.ranges.empty()) return;
        
        // check narrow front
        min_dist_ = 10.0;
        int center = scan_.ranges.size() / 2;
        int range = 10;
        
        for (int i = center - range; i <= center + range; ++i) {
            if (i >= 0 && i < (int)scan_.ranges.size()) {
                float r = scan_.ranges[i];
                if (r > 0.1 && r < min_dist_) {
                    min_dist_ = r;
                }
            }
        }
    }
    
    bool follow_srv(turtlebot3_explorer::FollowPath::Request& req,
                    turtlebot3_explorer::FollowPath::Response& res) {
        // clear old
        path_.clear();
        idx_ = 0;
        
        // store new
        for (size_t i = 0; i < req.path_x.size(); ++i) {
            path_.emplace_back(req.path_x[i], req.path_y[i]);
        }

        pub_path_viz(req.path_x, req.path_y);
        
        res.success = true;
        return true;
    }
    
    void control_cb(const ros::TimerEvent&) {
        if (!has_odom_ || path_.empty()) {
            cmd_pub_.publish(geometry_msgs::Twist());
            return;
        }
        
        // update index
        update_idx();
        
        // check complete
        if (is_done()) {
            complete();
            return;
        }
        
        // compute cmd
        geometry_msgs::Twist cmd = compute_cmd();
        cmd_pub_.publish(cmd);
        
        // progress
        std_msgs::Int32 prog;
        prog.data = idx_;
        prog_pub_.publish(prog);
    }
    
    void update_idx() {
        // advance waypoints
        while (idx_ < path_.size() - 1) {
            double d = euclideanDistance(
                pose_.position.x, pose_.position.y,
                path_[idx_].first, path_[idx_].second
            );
            
            if (d < pos_tol_) {
                idx_++;
            } else {
                break;
            }
        }
    }
    
    bool is_done() {
        if (idx_ >= path_.size() - 1) {
            double d = euclideanDistance(
                pose_.position.x, pose_.position.y,
                path_.back().first, path_.back().second
            );
            
            return d < goal_tol_;
        }
        return false;
    }
    
    void complete() {
        cmd_pub_.publish(geometry_msgs::Twist());
        path_.clear();
        
        std_msgs::Bool msg;
        msg.data = true;
        done_pub_.publish(msg);

        nav_msgs::Path empty;
        empty.header.stamp = ros::Time::now();
        empty.header.frame_id = "map";
        path_viz_pub_.publish(empty);
    }
    
    geometry_msgs::Twist compute_cmd() {
        geometry_msgs::Twist cmd;
        
        if (idx_ >= path_.size()) return cmd;
        
        // get target
        size_t tgt_idx = std::min(idx_ + lookahead_, path_.size() - 1);
        double tgt_x = path_[tgt_idx].first;
        double tgt_y = path_[tgt_idx].second;
        
        // path following
        double dx = tgt_x - pose_.position.x;
        double dy = tgt_y - pose_.position.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        
        double des_yaw = std::atan2(dy, dx);
        double cur_yaw = tf2::getYaw(pose_.orientation);
        double ang_err = normalizeAngle(des_yaw - cur_yaw);
        
        // speed control
        double spd_fac = 1.0;
        if (has_scan_ && min_dist_ < slow_dist_) {
            if (min_dist_ < crit_dist_) {
                // stop and turn
                cmd.linear.x = 0.0;
                cmd.angular.z = std::copysign(max_ang_, ang_err);
                return cmd;
            } else {
                // slow down
                spd_fac = (min_dist_ - crit_dist_) / (slow_dist_ - crit_dist_);
                spd_fac = std::max(0.3, spd_fac);
            }
        }
        
        // control
        if (std::abs(ang_err) > 0.5) {
            // turn in place
            cmd.linear.x = 0.0;
            cmd.angular.z = std::copysign(max_ang_, ang_err);
        } else {
            // forward with turn
            cmd.linear.x = max_lin_ * spd_fac;
            cmd.angular.z = ang_err * 1.5;
        }
        
        // clamp
        cmd.linear.x = std::max(0.0, std::min(cmd.linear.x, max_lin_));
        cmd.angular.z = std::max(-max_ang_, std::min(cmd.angular.z, max_ang_));
        
        return cmd;
    }

    void pub_path_viz(const std::vector<double>& x, const std::vector<double>& y) {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";
        
        for (size_t i = 0; i < x.size(); ++i) {
            geometry_msgs::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = x[i];
            pose.pose.position.y = y[i];
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        
        path_viz_pub_.publish(path_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower");
    ros::NodeHandle nh("~");
    
    std::string name;
    if (!nh.getParam("robot_name", name)) {
        return 1;
    }
    
    PathFollower follower(nh, name);
    ros::spin();
    
    return 0;
}