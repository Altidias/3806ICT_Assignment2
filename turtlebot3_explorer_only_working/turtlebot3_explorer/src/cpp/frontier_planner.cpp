#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mutex>

class FrontierPlanner {
public:
    FrontierPlanner(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
        : nh_(nh) {
        private_nh.param<std::string>("frontiers_topic", frontiers_topic_, "/frontiers");
        private_nh.param<std::string>("goal_topic", goal_topic_, "/exploration_goal");
        private_nh.param<std::string>("odom_topic", odom_topic_, "/odom");

        ROS_INFO("Subscribing to frontiers topic: %s", frontiers_topic_.c_str());
        ROS_INFO("Publishing goals to topic: %s", goal_topic_.c_str());
        ROS_INFO("Subscribing to odom topic: %s", odom_topic_.c_str());

        frontiers_sub_ = nh_.subscribe(frontiers_topic_, 10, &FrontierPlanner::frontiersCallback, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 10, &FrontierPlanner::odomCallback, this);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 10);
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        robot_pose_ = msg->pose.pose;
    }

    void frontiersCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        if (msg->poses.empty()) {
            ROS_INFO("No frontiers received");
            return;
        }

        // Select the closest frontier
        double min_dist = std::numeric_limits<double>::max();
        geometry_msgs::Pose selected_pose;
        for (const auto& pose : msg->poses) {
            double dist = std::hypot(
                pose.position.x - robot_pose_.position.x,
                pose.position.y - robot_pose_.position.y
            );
            if (dist < min_dist) {
                min_dist = dist;
                selected_pose = pose;
            }
        }

        ROS_INFO("Selected frontier at (%f, %f) with distance %f",
                 selected_pose.position.x, selected_pose.position.y, min_dist);

        // Publish the goal
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose = selected_pose;
        goal_pub_.publish(goal);
    }

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

int main(int argc, char** argv) {
    ros::init(argc, argv, "frontier_planner");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    FrontierPlanner planner(nh, private_nh);
    ros::spin();
    return 0;
}
