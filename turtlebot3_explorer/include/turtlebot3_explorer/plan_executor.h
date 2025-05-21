#ifndef PLAN_EXECUTOR_H
#define PLAN_EXECUTOR_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <turtlebot3_explorer/ExecutionStatus.h>
#include <mutex>
#include <cmath>

class PlanExecutor {
public:
    PlanExecutor(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~PlanExecutor();

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher status_pub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber scan_sub_;
    tf::TransformListener tf_listener_;
    ros::Timer control_timer_;
    
    std::string cmd_vel_topic_;
    std::string goal_topic_;
    std::string robot_frame_;
    std::string map_frame_;
    double goal_tolerance_;
    double max_linear_vel_;
    double max_angular_vel_;
    double obstacle_threshold_;
    
    // State
    geometry_msgs::PoseStamped current_goal_;
    sensor_msgs::LaserScan latest_scan_;
    std::mutex goal_mutex_;
    std::mutex scan_mutex_;
    bool have_goal_;
    bool have_scan_;
    
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void controlLoop(const ros::TimerEvent& event);
    bool getRobotPose(tf::StampedTransform& transform);
    bool checkObstacles();
};

#endif // PLAN_EXECUTOR_H