#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <turtlebot3_explorer/SelectFrontiers.h>

class Coordinator {
public:
    Coordinator(ros::NodeHandle& nh) : nh_(nh) {
        map_sub_ = nh_.subscribe("/map", 10, &Coordinator::map_cb, this);
        frontiers_sub_ = nh_.subscribe("/frontiers", 10, &Coordinator::frontiers_cb, this);
        status_pub_ = nh_.advertise<std_msgs::String>("/frontier_status", 10);
        
        // init robot positions
        robot_poses_.poses.resize(2); // 2 robots
        robot_ids_.push_back("robot1");
        robot_ids_.push_back("robot2");
        
        ROS_INFO("Coordinator ready");
    }

private:
    void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        map_received_ = true;
    }

    void frontiers_cb(const geometry_msgs::PoseArray::ConstPtr& msg) {
        frontiers_ = *msg;
        assign();
    }

    void assign() {
        if (!map_received_ || frontiers_.poses.empty()) {
            return;
        }

        turtlebot3_explorer::SelectFrontiers srv;
        srv.request.robot_ids = robot_ids_;
        srv.request.robot_poses = robot_poses_;
        srv.request.frontiers = frontiers_;

        // call llm service
        if (ros::service::call("/llm_select_frontiers", srv)) {
            if (srv.response.success) {
                for (size_t i = 0; i < srv.response.frontier_indices.size(); ++i) {
                    int idx = srv.response.frontier_indices[i];
                    if (idx >= 0 && idx < frontiers_.poses.size()) {
                        ROS_INFO("Robot %s -> frontier %d: %s",
                                robot_ids_[i].c_str(), idx,
                                srv.response.reasoning[i].c_str());
                    }
                }
            } else {
                ROS_ERROR("LLM selection failed!");
            }
        } else {
            ROS_ERROR("Service call failed");
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber frontiers_sub_;
    ros::Publisher status_pub_;
    
    bool map_received_ = false;
    geometry_msgs::PoseArray frontiers_;
    geometry_msgs::PoseArray robot_poses_;
    std::vector<std::string> robot_ids_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "frontier_planner");
    ros::NodeHandle nh;
    
    Coordinator coordinator(nh);
    ros::spin();
    
    return 0;
}