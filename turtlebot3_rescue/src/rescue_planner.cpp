#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <turtlebot3_rescue/OptimizePath.h>

class RescueNavigator {
public:
  RescueNavigator(ros::NodeHandle& nh) : nh_(nh), current_target_idx_(0) {
    map_sub_ = nh_.subscribe("/map", 10, &RescueNavigator::mapCallback, this);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    servo_pub_ = nh_.advertise<std_msgs::Bool>("/servo_control", 10);
    status_pub_ = nh_.advertise<std_msgs::String>("/rescue_status", 10);

    geometry_msgs::Pose pose1, pose2, pose3;
    pose1.position.x = 2.0; pose1.position.y = 1.0; pose1.orientation.w = 1.0;
    pose2.position.x = 4.0; pose2.position.y = 3.0; pose2.orientation.w = 1.0;
    pose3.position.x = 6.0; pose3.position.y = 2.0; pose3.orientation.w = 1.0;
    all_targets_.poses.push_back(pose1);
    all_targets_.poses.push_back(pose2);
    all_targets_.poses.push_back(pose3);

    ROS_INFO("Initialized with %zu targets", all_targets_.poses.size());
} 

private:
   void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_received_ = true;
    ROS_INFO("Map received, triggering goal selection...");
    selectNextTarget();
}

void selectNextTarget() {
    ROS_INFO("Entering selectNextTarget, current_target_idx_: %zu, targets size: %zu", current_target_idx_, all_targets_.poses.size());
    if (current_target_idx_ >= all_targets_.poses.size()) {
        std_msgs::String status_msg;
        status_msg.data = "All targets reached, mission complete";
        status_pub_.publish(status_msg);
        ROS_INFO("%s", status_msg.data.c_str());
        return;
    }

    turtlebot3_rescue::OptimizePath srv;
    srv.request.targets = all_targets_;
    ROS_INFO("Calling LLM service with %zu targets", all_targets_.poses.size());
    if (ros::service::call("/llm_optimize_path", srv)) {
        ROS_INFO("LLM service returned %zu targets", srv.response.optimized_targets.poses.size());
        if (srv.response.optimized_targets.poses.empty()) {
            ROS_ERROR("LLM returned no targets!");
            return;
        }

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose = srv.response.optimized_targets.poses[0];
        goal_pub_.publish(goal);
        ROS_INFO("Published goal to (x=%f, y=%f)", goal.pose.position.x, goal.pose.position.y);

        std_msgs::Bool servo_msg;
        servo_msg.data = true;
        servo_pub_.publish(servo_msg);

        std_msgs::String status_msg;
        status_msg.data = "Selected target at (x=" + std::to_string(goal.pose.position.x) +
                          ", y=" + std::to_string(goal.pose.position.y) + ")";
        status_pub_.publish(status_msg);
        ROS_INFO("%s", status_msg.data.c_str());

        for (auto it = all_targets_.poses.begin(); it != all_targets_.poses.end(); ++it) {
            if (std::abs(it->position.x - goal.pose.position.x) < 0.01 &&
                std::abs(it->position.y - goal.pose.position.y) < 0.01) {
                all_targets_.poses.erase(it);
                break;
            }
        }

        current_target_idx_++;
        ROS_INFO("Sleeping for 2 seconds before next target selection...");
        ros::Duration(2.0).sleep();
        ROS_INFO("Finished sleeping, proceeding to next target...");
        selectNextTarget();
    } else {
        ROS_ERROR("Failed to call LLM service /llm_optimize_path");
    }
}
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Publisher goal_pub_;
    ros::Publisher servo_pub_;
    ros::Publisher status_pub_;
    geometry_msgs::PoseArray all_targets_;
    bool map_received_ = false;
    size_t current_target_idx_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rescue_navigator");
    ros::NodeHandle nh;
    RescueNavigator navigator(nh);
    ros::spin();
    return 0;
}
