#!/usr/bin/env python3

import rospy
from turtlebot3_explorer.msg import ExecutionStatus
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
import threading

class ExplorationMonitor:
    def __init__(self):
        rospy.init_node('exploration_monitor')
        
        self.robot_status = {}
        self.robot_paths = {}
        self.frontiers = []
        self.lock = threading.Lock()
        
        # Robot colors for visualization
        self.robot_colors = {
            'robot1': ColorRGBA(1.0, 0.0, 0.0, 1.0),  # Red
            'robot2': ColorRGBA(0.0, 1.0, 0.0, 1.0),  # Green
            'robot3': ColorRGBA(0.0, 0.0, 1.0, 1.0),  # Blue
        }
        
        # Subscribers
        self.status_subs = {}
        robot_namespaces = rospy.get_param('~robot_namespaces', ['robot1', 'robot2'])
        
        for robot_id in robot_namespaces:
            self.status_subs[robot_id] = rospy.Subscriber(
                f'/{robot_id}/execution_status', ExecutionStatus,
                lambda msg, rid=robot_id: self.status_callback(msg, rid)
            )
            
        self.frontiers_sub = rospy.Subscriber('/frontiers', PoseArray, self.frontiers_callback)
        
        # Publishers
        self.viz_pub = rospy.Publisher('/exploration_visualization', MarkerArray, queue_size=1)
        
        # Timer for visualization updates
        self.viz_timer = rospy.Timer(rospy.Duration(0.5), self.visualization_update)
        
        # Timer for console output
        self.console_timer = rospy.Timer(rospy.Duration(2.0), self.console_update)
        
        rospy.loginfo("Exploration Monitor started")
        
    def status_callback(self, msg, robot_id):
        with self.lock:
            self.robot_status[robot_id] = msg
            
    def frontiers_callback(self, msg):
        with self.lock:
            self.frontiers = msg.poses
            
    def console_update(self, event):
        """Print status to console"""
        with self.lock:
            print("\n" + "="*60)
            print("EXPLORATION STATUS UPDATE")
            print("="*60)
            
            # Robot status
            for robot_id, status in sorted(self.robot_status.items()):
                print(f"\n{robot_id}:")
                print(f"  State: {status.status}")
                
                if status.current_pose:
                    print(f"  Position: ({status.current_pose.position.x:.2f}, "
                          f"{status.current_pose.position.y:.2f})")
                
                if status.target_pose:
                    print(f"  Target: ({status.target_pose.position.x:.2f}, "
                          f"{status.target_pose.position.y:.2f})")
                
                if status.progress_percentage > 0:
                    print(f"  Progress: {status.progress_percentage:.1f}%")
                    
            # Frontier status
            print(f"\nActive Frontiers: {len(self.frontiers)}")
            
            print("="*60)
            
    def visualization_update(self, event):
        """Update visualization markers"""
        marker_array = MarkerArray()
        marker_id = 0
        
        with self.lock:
            # Robot position markers
            for robot_id, status in self.robot_status.items():
                if not status.current_pose:
                    continue
                    
                # Robot position marker
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "robot_positions"
                marker.id = marker_id
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                
                marker.pose = status.current_pose
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.5
                
                marker.color = self.robot_colors.get(robot_id, ColorRGBA(0.5, 0.5, 0.5, 1.0))
                
                marker_array.markers.append(marker)
                marker_id += 1
                
                # Robot label
                label = Marker()
                label.header.frame_id = "map"
                label.header.stamp = rospy.Time.now()
                label.ns = "robot_labels"
                label.id = marker_id
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                
                label.pose = status.current_pose
                label.pose.position.z += 0.7
                label.scale.z = 0.3
                label.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
                label.text = f"{robot_id}\n{status.status}"
                
                marker_array.markers.append(label)
                marker_id += 1
                
                # Target marker if exists
                if status.target_pose:
                    target = Marker()
                    target.header.frame_id = "map"
                    target.header.stamp = rospy.Time.now()
                    target.ns = "robot_targets"
                    target.id = marker_id
                    target.type = Marker.SPHERE
                    target.action = Marker.ADD
                    
                    target.pose = status.target_pose
                    target.scale.x = 0.2
                    target.scale.y = 0.2
                    target.scale.z = 0.2
                    
                    target.color = self.robot_colors.get(robot_id, ColorRGBA(0.5, 0.5, 0.5, 0.5))
                    target.color.a = 0.5
                    
                    marker_array.markers.append(target)
                    marker_id += 1
                    
                    # Line from robot to target
                    line = Marker()
                    line.header.frame_id = "map"
                    line.header.stamp = rospy.Time.now()
                    line.ns = "robot_paths"
                    line.id = marker_id
                    line.type = Marker.LINE_STRIP
                    line.action = Marker.ADD
                    
                    line.points.append(status.current_pose.position)
                    line.points.append(status.target_pose.position)
                    
                    line.scale.x = 0.05
                    line.color = self.robot_colors.get(robot_id, ColorRGBA(0.5, 0.5, 0.5, 0.5))
                    line.color.a = 0.5
                    
                    marker_array.markers.append(line)
                    marker_id += 1
                    
            # Frontier markers
            for i, frontier_pose in enumerate(self.frontiers):
                frontier = Marker()
                frontier.header.frame_id = "map"
                frontier.header.stamp = rospy.Time.now()
                frontier.ns = "frontiers"
                frontier.id = marker_id
                frontier.type = Marker.SPHERE
                frontier.action = Marker.ADD
                
                frontier.pose = frontier_pose
                frontier.scale.x = 0.3
                frontier.scale.y = 0.3
                frontier.scale.z = 0.3
                
                frontier.color = ColorRGBA(1.0, 1.0, 0.0, 0.8)  # Yellow
                
                marker_array.markers.append(frontier)
                marker_id += 1
                
        self.viz_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        monitor = ExplorationMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass