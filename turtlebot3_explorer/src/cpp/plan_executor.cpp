#include <turtlebot3_explorer/plan_executor.h>

PlanExecutor::PlanExecutor(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), have_goal_(false), have_scan_(false)
{

}

PlanExecutor::~PlanExecutor() {
    // stop the robot
    geometry_msgs::Twist cmd_vel;
    cmd_vel_pub_.publish(cmd_vel);
}

void PlanExecutor::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    // new goal recieved 
    std::lock_guard<std::mutex> lock(goal_mutex_);
    current_goal_ = *goal;
    have_goal_ = true;
}

void PlanExecutor::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = *scan;
    have_scan_ = true;
}

bool PlanExecutor::getRobotPose(tf::StampedTransform& transform) {
    
}

bool PlanExecutor::checkObstacles() {
    
}

void PlanExecutor::controlLoop(const ros::TimerEvent& event) {
    // check if we have a goal
    geometry_msgs::PoseStamped current_goal;
    bool have_goal;
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        current_goal = current_goal_;
        have_goal = have_goal_;
    }
    
    if (!have_goal) {
        // no goal, stop moving
        geometry_msgs::Twist cmd_vel;
        cmd_vel_pub_.publish(cmd_vel);
        return;
    }
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "plan_executor");
    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    PlanExecutor executor(nh, private_nh);
    
    ros::spin();
    
    return 0;
}