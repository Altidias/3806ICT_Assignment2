#ifndef FRONTIER_PLANNER_H_
#define FRONTIER_PLANNER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mutex>

class FrontierPlanner {
public:
    FrontierPlanner(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~FrontierPlanner() = default;

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void frontiersCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber frontiers_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher goal_pub_;
    std::string frontiers_topic_;
    std::string goal_topic_;
    std::string odom_topic_;
    geometry_msgs::Pose robot_pose_;
    std::mutex pose_mutex_;
};

#endif // FRONTIER_PLANNER_H_
