#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <turtlebot3_explorer/SensorData.h>
#include <mutex>

class SensorInterface {
public:
    SensorInterface(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~SensorInterface();

private:
    ros::NodeHandle nh_;
    ros::Publisher sensor_data_pub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber map_sub_;
    tf::TransformListener tf_listener_;
    ros::Timer publish_timer_;

    std::string map_frame_;
    std::string robot_frame_;
    
    sensor_msgs::LaserScan latest_scan_;
    nav_msgs::OccupancyGrid latest_map_;
    std::mutex scan_mutex_;
    std::mutex map_mutex_;
    bool have_scan_;
    bool have_map_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void publishSensorData(const ros::TimerEvent& event);
    bool getRobotPose(tf::StampedTransform& transform);
};

#endif // SENSOR_INTERFACE_H