#include <turtlebot3_explorer/sensor_interface.h>

SensorInterface::SensorInterface(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), have_scan_(false), have_map_(false)
{
    private_nh.param<std::string>("map_frame", map_frame_, "map");
    private_nh.param<std::string>("robot_frame", robot_frame_, "base_footprint");
    
    sensor_data_pub_ = nh_.advertise<turtlebot3_explorer::SensorData>("/sensor_data", 1);
    
    scan_sub_ = nh_.subscribe("/scan", 1, &SensorInterface::scanCallback, this);
    map_sub_ = nh_.subscribe("/map", 1, &SensorInterface::mapCallback, this);
    
    publish_timer_ = nh_.createTimer(ros::Duration(0.1), &SensorInterface::publishSensorData, this);
    
}

SensorInterface::~SensorInterface() {
}

void SensorInterface::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = *scan;
    have_scan_ = true;
}

void SensorInterface::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_map_ = *map;
    have_map_ = true;
}

bool SensorInterface::getRobotPose(tf::StampedTransform& transform) {
    try {
        tf_listener_.waitForTransform(map_frame_, robot_frame_, ros::Time(0), ros::Duration(0.5));
        tf_listener_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), transform);
        return true;
    } catch (tf::TransformException& ex) {
        return false;
    }
}

void SensorInterface::publishSensorData(const ros::TimerEvent& event) {
    if (!have_scan_ || !have_map_) {
        return;
    }
    
    tf::StampedTransform robot_transform;
    if (!getRobotPose(robot_transform)) {
        return;
    }
    
    // sensor data message
    turtlebot3_explorer::SensorData sensor_data;
    sensor_data.header.stamp = ros::Time::now();
    sensor_data.header.frame_id = map_frame_;
    
    // set robot pose
    sensor_data.robot_pose.position.x = robot_transform.getOrigin().x();
    sensor_data.robot_pose.position.y = robot_transform.getOrigin().y();
    sensor_data.robot_pose.position.z = robot_transform.getOrigin().z();
    
    // quaternion to pose
    tf::Quaternion q = robot_transform.getRotation();
    sensor_data.robot_pose.orientation.x = q.x();
    sensor_data.robot_pose.orientation.y = q.y();
    sensor_data.robot_pose.orientation.z = q.z();
    sensor_data.robot_pose.orientation.w = q.w();
    
    // add scan and map data
    {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        sensor_data.scan = latest_scan_;
    }
    
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        sensor_data.map_data = latest_map_;
    }
    
    sensor_data_pub_.publish(sensor_data);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_interface");
    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    SensorInterface interface(nh, private_nh);
    
    ros::spin();
    
    return 0;
}