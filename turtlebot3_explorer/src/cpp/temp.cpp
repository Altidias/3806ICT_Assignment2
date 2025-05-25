#include <turtlebot3_explorer/plan_executor.h>
#include <angles/angles.h>

PlanExecutor::PlanExecutor(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), have_goal_(false), have_scan_(false), tf_listener_() {
    private_nh.param<std::string>("goal_topic", goal_topic_, "/exploration_goal");
    private_nh.param<std::string>("scan_topic", scan_topic_, "/scan");
    private_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");

    ROS_INFO("Subscribing to goal topic: %s", goal_topic_.c_str());
    ROS_INFO("Subscribing to scan topic: %s", scan_topic_.c_str());
    ROS_INFO("Publishing to cmd_vel topic: %s", cmd_vel_topic_.c_str());

    goal_sub_ = nh_.subscribe(goal_topic_, 10, &PlanExecutor::goalCallback, this);
    scan_sub_ = nh_.subscribe(scan_topic_, 10, &PlanExecutor::scanCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);

    control_timer_ = nh_.createTimer(ros::Duration(0.1), &PlanExecutor::controlLoop, this);
}

PlanExecutor::~PlanExecutor() {
    // Stop the robot
    geometry_msgs::Twist cmd_vel;
    cmd_vel_pub_.publish(cmd_vel);
}

void PlanExecutor::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    current_goal_ = *goal;
    have_goal_ = true;
    ROS_INFO("Received new goal at (%f, %f)", goal->pose.position.x, goal->pose.position.y);
}

void PlanExecutor::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = *scan;
    have_scan_ = true;
}

bool PlanExecutor::getRobotPose(tf::StampedTransform& transform) {
    try {
        tf_listener_.lookupTransform("map", "base_link", ros::Time(0), transform);
        return true;
    } catch (tf::TransformException& ex) {
        ROS_WARN("Failed to get robot pose: %s", ex.what());
        return false;
    }
}

bool PlanExecutor::checkObstacles() {
    if (!have_scan_) {
        ROS_WARN("No laser scan available for obstacle checking");
        return false;
    }

    // Check for obstacles in front (e.g., within 0.3 meters, central 60 degrees)
    float min_obstacle_distance = 0.3; // meters
    int center_idx = latest_scan_.ranges.size() / 2;
    int angle_range = 30 * (latest_scan_.ranges.size() / 360); // 60 degrees total

    for (int i = center_idx - angle_range; i <= center_idx + angle_range; ++i) {
        if (i >= 0 && i < latest_scan_.ranges.size()) {
            float range = latest_scan_.ranges[i];
            if (std::isfinite(range) && range < min_obstacle_distance) {
                ROS_WARN("Obstacle detected at %f meters", range);
                return true; // Obstacle detected
            }
        }
    }
    return false; // No obstacles
}

void PlanExecutor::controlLoop(const ros::TimerEvent& event) {
    // Check if we have a goal
    geometry_msgs::PoseStamped current_goal;
    bool have_goal;
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        current_goal = current_goal_;
        have_goal = have_goal_;
    }

    if (!have_goal) {
        // No goal, stop moving
        geometry_msgs::Twist cmd_vel;
        cmd_vel_pub_.publish(cmd_vel);
        return;
    }

    // Get the robot's current pose
    tf::StampedTransform transform;
    if (!getRobotPose(transform)) {
        ROS_WARN("Cannot navigate without robot pose");
        return;
    }

    // Extract robot's position and orientation
    double robot_x = transform.getOrigin().x();
    double robot_y = transform.getOrigin().y();
    tf::Quaternion q(
        transform.getRotation().x(),
        transform.getRotation().y(),
        transform.getRotation().z(),
        transform.getRotation().w()
    );
    double robot_yaw = tf::getYaw(q);

    // Calculate the distance and angle to the goal
    double goal_x = current_goal.pose.position.x;
    double goal_y = current_goal.pose.position.y;
    double dx = goal_x - robot_x;
    double dy = goal_y - robot_y;
    double distance_to_goal = std::hypot(dx, dy);
    double angle_to_goal = std::atan2(dy, dx);
    double angle_diff = angles::shortest_angular_distance(robot_yaw, angle_to_goal);

    // Check if we've reached the goal (within 0.1 meters)
    if (distance_to_goal < 0.1) {
        ROS_INFO("Reached goal at (%f, %f)", goal_x, goal_y);
        std::lock_guard<std::mutex> lock(goal_mutex_);
        have_goal_ = false;
        geometry_msgs::Twist cmd_vel;
        cmd_vel_pub_.publish(cmd_vel);
        return;
    }

    // Check for obstacles
    if (checkObstacles()) {
        // Stop and turn to avoid obstacle
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.5; // Turn right to avoid
        cmd_vel_pub_.publish(cmd_vel);
        return;
    }

    // Move toward the goal
    geometry_msgs::Twist cmd_vel;
    // Proportional control for linear and angular velocity
    cmd_vel.linear.x = std::min(0.2, 0.5 * distance_to_goal); // Max speed 0.2 m/s
    cmd_vel.angular.z = std::min(0.5, 2.0 * angle_diff); // Max angular speed 0.5 rad/s

    cmd_vel_pub_.publish(cmd_vel);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "plan_executor");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    PlanExecutor executor(nh, private_nh);
    ros::spin();
    return 0;
}
