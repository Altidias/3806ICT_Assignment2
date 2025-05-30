#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import csv
import os
from collections import defaultdict, deque
from threading import Lock

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool, String, Float32MultiArray
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import PointCloud2

class Tracker:
    def __init__(self):
        rospy.init_node('performance_tracker')
        
        # params
        self.robot_names = rospy.get_param('~robot_names', ['robot1', 'robot2'])
        self.update_rate = rospy.get_param('~update_rate', 1.0)
        self.export_path = rospy.get_param('~export_path', '/tmp/performance_data')
        self.enable_live_plot = rospy.get_param('~enable_live_plot', False)
        self.window_size = rospy.get_param('~metrics_window_size', 100)
        
        # create export dir
        os.makedirs(self.export_path, exist_ok=True)
        
        # state tracking
        self.lock = Lock()
        self.start_time = rospy.Time.now()
        self.robot_poses = {}
        self.robot_paths = {robot: [] for robot in self.robot_names}
        self.map_data = None
        self.frontiers = []
        self.explored_area = 0.0
        self.total_distance = {robot: 0.0 for robot in self.robot_names}
        self.frontier_history = deque(maxlen=self.window_size)
        self.exploration_history = deque(maxlen=self.window_size)
        
        # performance metrics
        self.metrics = {
            'exploration_rate': deque(maxlen=self.window_size),
            'frontier_reduction_rate': deque(maxlen=self.window_size),
            'robot_separation': deque(maxlen=self.window_size),
            'coverage_efficiency': deque(maxlen=self.window_size),
            'path_optimality': deque(maxlen=self.window_size),
            'coordination_index': deque(maxlen=self.window_size)
        }
        
        # subs
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb)
        self.frontiers_sub = rospy.Subscriber('/frontiers', PoseArray, self.front_cb)
        self.exploration_complete_sub = rospy.Subscriber('/exploration_complete', Bool, self.complete_cb)
        
        # robot subs
        for robot in self.robot_names:
            rospy.Subscriber(f'/{robot}/odom', Odometry, 
                           lambda msg, r=robot: self.odom_cb(msg, r))
        
        # pubs
        self.metrics_pub = rospy.Publisher('/performance_metrics', Float32MultiArray, queue_size=1)
        self.coverage_viz_pub = rospy.Publisher('/coverage_visualization', MarkerArray, queue_size=1)
        self.efficiency_viz_pub = rospy.Publisher('/efficiency_visualization', MarkerArray, queue_size=1)
        
        # data export files
        self.init_export()
        
        # timer for metric computation
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.compute)
        
        # live plotting setup
        if self.enable_live_plot:
            self.setup_plot()
    
    def init_export(self):
        """init csv files"""
        timestamp = rospy.Time.now().secs
        
        # main metrics file
        self.metrics_file = open(f'{self.export_path}/metrics_{timestamp}.csv', 'w', newline='')
        self.metrics_writer = csv.writer(self.metrics_file)
        self.metrics_writer.writerow([
            'timestamp', 'elapsed_time', 'explored_area', 'frontier_count',
            'exploration_rate', 'frontier_reduction_rate', 'robot_separation',
            'coverage_efficiency', 'path_optimality', 'coordination_index'
        ])
        
        # robot-specific data
        self.robot_data_files = {}
        self.robot_data_writers = {}
        for robot in self.robot_names:
            filename = f'{self.export_path}/robot_{robot}_{timestamp}.csv'
            file_obj = open(filename, 'w', newline='')
            writer = csv.writer(file_obj)
            writer.writerow(['timestamp', 'x', 'y', 'total_distance', 'instantaneous_speed'])
            self.robot_data_files[robot] = file_obj
            self.robot_data_writers[robot] = writer
    
    def map_cb(self, msg):
        """process map updates and calc explored area"""
        with self.lock:
            self.map_data = msg
            
            # calc explored area
            free_cells = sum(1 for cell in msg.data if cell == 0)
            total_cells = len([cell for cell in msg.data if cell != -1])  # exclude unknown
            
            if total_cells > 0:
                self.explored_area = (free_cells / len(msg.data)) * 100.0  # percentage
    
    def front_cb(self, msg):
        """track frontier points"""
        with self.lock:
            self.frontiers = [(pose.position.x, pose.position.y) for pose in msg.poses]
    
    def odom_cb(self, msg, robot_name):
        """track robot positions and calc distances"""
        with self.lock:
            current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            
            # calc distance traveled
            if robot_name in self.robot_poses:
                prev_pose = self.robot_poses[robot_name]
                distance = np.sqrt((current_pose[0] - prev_pose[0])**2 + 
                                 (current_pose[1] - prev_pose[1])**2)
                self.total_distance[robot_name] += distance
            
            self.robot_poses[robot_name] = current_pose
            self.robot_paths[robot_name].append((current_pose, rospy.Time.now()))
            
            # limit path history
            if len(self.robot_paths[robot_name]) > 1000:
                self.robot_paths[robot_name] = self.robot_paths[robot_name][-500:]
    
    def complete_cb(self, msg):
        """handle exploration completion"""
        if msg.data:
            self.export_final()
    
    def compute(self, event):
        """compute performance metrics"""
        with self.lock:
            current_time = rospy.Time.now()
            elapsed_time = (current_time - self.start_time).to_sec()
            
            if elapsed_time < 1.0:  # wait for system to stabilize
                return
            
            # exploration rate (area per second)
            if len(self.exploration_history) > 0:
                time_diff = elapsed_time - self.exploration_history[-1][1] if self.exploration_history else 1.0
                area_diff = self.explored_area - self.exploration_history[-1][0] if self.exploration_history else 0.0
                exploration_rate = area_diff / max(time_diff, 0.1)
            else:
                exploration_rate = 0.0
            
            self.exploration_history.append((self.explored_area, elapsed_time))
            self.metrics['exploration_rate'].append(exploration_rate)
            
            # frontier reduction rate
            if len(self.frontier_history) > 0:
                frontier_diff = len(self.frontiers) - self.frontier_history[-1][0]
                frontier_rate = -frontier_diff / max(time_diff, 0.1)  # negative because we want reduction
            else:
                frontier_rate = 0.0
            
            self.frontier_history.append((len(self.frontiers), elapsed_time))
            self.metrics['frontier_reduction_rate'].append(frontier_rate)
            
            # robot separation (coordination metric)
            robot_separation = self.calc_separation()
            self.metrics['robot_separation'].append(robot_separation)
            
            # coverage efficiency (explored area / total distance)
            total_dist = sum(self.total_distance.values())
            coverage_efficiency = self.explored_area / max(total_dist, 0.1)
            self.metrics['coverage_efficiency'].append(coverage_efficiency)
            
            # path optimality (straight-line vs actual distance ratio)
            path_optimality = self.calc_optimality()
            self.metrics['path_optimality'].append(path_optimality)
            
            # coordination index (how well robots are distributed)
            coordination_index = self.calc_coordination()
            self.metrics['coordination_index'].append(coordination_index)
            
            # export metrics
            self.export_current(current_time, elapsed_time)
            
            # publish metrics for real-time monitoring
            self.pub_metrics()
            
            # update visualizations
            self.pub_viz()
    
    def calc_separation(self):
        """calc average distance between robots"""
        if len(self.robot_poses) < 2:
            return 0.0
        
        positions = list(self.robot_poses.values())
        total_separation = 0.0
        count = 0
        
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                dist = np.sqrt((positions[i][0] - positions[j][0])**2 + 
                              (positions[i][1] - positions[j][1])**2)
                total_separation += dist
                count += 1
        
        return total_separation / max(count, 1)
    
    def calc_optimality(self):
        """calc how optimal the paths are (straight-line vs actual)"""
        if not self.robot_paths:
            return 1.0
        
        optimality_scores = []
        
        for robot, path in self.robot_paths.items():
            if len(path) < 2:
                continue
            
            # calc straight-line distance from start to current
            start_pos = path[0][0]
            current_pos = path[-1][0]
            straight_line_dist = np.sqrt((current_pos[0] - start_pos[0])**2 + 
                                       (current_pos[1] - start_pos[1])**2)
            
            # actual distance traveled
            actual_dist = self.total_distance[robot]
            
            if actual_dist > 0:
                optimality = min(straight_line_dist / actual_dist, 1.0)
                optimality_scores.append(optimality)
        
        return np.mean(optimality_scores) if optimality_scores else 1.0
    
    def calc_coordination(self):
        """calc how well robots are coordinated (avoiding overlap)"""
        if len(self.robot_poses) < 2 or not self.frontiers:
            return 1.0
        
        # calc how well robots are distributed among frontiers
        robot_positions = list(self.robot_poses.values())
        
        # find closest frontier for each robot
        robot_frontiers = []
        for robot_pos in robot_positions:
            min_dist = float('inf')
            closest_frontier = None
            for frontier in self.frontiers:
                dist = np.sqrt((robot_pos[0] - frontier[0])**2 + 
                              (robot_pos[1] - frontier[1])**2)
                if dist < min_dist:
                    min_dist = dist
                    closest_frontier = frontier
            if closest_frontier:
                robot_frontiers.append(closest_frontier)
        
        # check for duplicate frontier assignments
        unique_frontiers = set(robot_frontiers)
        coordination_score = len(unique_frontiers) / len(robot_frontiers) if robot_frontiers else 1.0
        
        return coordination_score
    
    def export_current(self, current_time, elapsed_time):
        """export current metrics to csv"""
        metrics_values = [
            current_time.to_sec(),
            elapsed_time,
            self.explored_area,
            len(self.frontiers),
            self.metrics['exploration_rate'][-1] if self.metrics['exploration_rate'] else 0.0,
            self.metrics['frontier_reduction_rate'][-1] if self.metrics['frontier_reduction_rate'] else 0.0,
            self.metrics['robot_separation'][-1] if self.metrics['robot_separation'] else 0.0,
            self.metrics['coverage_efficiency'][-1] if self.metrics['coverage_efficiency'] else 0.0,
            self.metrics['path_optimality'][-1] if self.metrics['path_optimality'] else 0.0,
            self.metrics['coordination_index'][-1] if self.metrics['coordination_index'] else 0.0
        ]
        
        self.metrics_writer.writerow(metrics_values)
        self.metrics_file.flush()
        
        # export robot-specific data
        for robot in self.robot_names:
            if robot in self.robot_poses:
                pos = self.robot_poses[robot]
                
                # calc instantaneous speed
                speed = 0.0
                if len(self.robot_paths[robot]) >= 2:
                    recent_poses = self.robot_paths[robot][-2:]
                    time_diff = (recent_poses[1][1] - recent_poses[0][1]).to_sec()
                    if time_diff > 0:
                        pos_diff = np.sqrt((recent_poses[1][0][0] - recent_poses[0][0][0])**2 + 
                                         (recent_poses[1][0][1] - recent_poses[0][0][1])**2)
                        speed = pos_diff / time_diff
                
                robot_data = [
                    current_time.to_sec(),
                    pos[0], pos[1],
                    self.total_distance[robot],
                    speed
                ]
                
                self.robot_data_writers[robot].writerow(robot_data)
                self.robot_data_files[robot].flush()
    
    def pub_metrics(self):
        """publish metrics for real-time monitoring"""
        msg = Float32MultiArray()
        
        # pack current metrics
        current_metrics = []
        for metric_name, values in self.metrics.items():
            current_metrics.append(values[-1] if values else 0.0)
        
        msg.data = [
            self.explored_area,
            len(self.frontiers),
            len(self.robot_poses)
        ] + current_metrics
        
        self.metrics_pub.publish(msg)
    
    def pub_viz(self):
        """publish visualization markers"""
        # coverage visualization
        coverage_markers = MarkerArray()
        
        # robot path trails
        for i, (robot, path) in enumerate(self.robot_paths.items()):
            if len(path) < 2:
                continue
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "robot_trails"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            marker.scale.x = 0.05
            
            # color code by robot
            if robot == "robot1":
                marker.color.r = 1.0
                marker.color.a = 0.7
            else:
                marker.color.g = 1.0
                marker.color.a = 0.7
            
            for pose, timestamp in path:
                point = Point()
                point.x = pose[0]
                point.y = pose[1]
                point.z = 0.05
                marker.points.append(point)
            
            coverage_markers.markers.append(marker)
        
        # efficiency visualization (robot separation circles)
        efficiency_markers = MarkerArray()
        
        for i, (robot, pos) in enumerate(self.robot_poses.items()):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "separation_circles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # circle size based on average separation
            avg_separation = self.metrics['robot_separation'][-1] if self.metrics['robot_separation'] else 1.0
            marker.scale.x = avg_separation
            marker.scale.y = avg_separation
            marker.scale.z = 0.1
            
            marker.color.b = 1.0
            marker.color.a = 0.3
            
            efficiency_markers.markers.append(marker)
        
        self.coverage_viz_pub.publish(coverage_markers)
        self.efficiency_viz_pub.publish(efficiency_markers)
    
    def export_final(self):
        """export final performance summary"""
        summary_file = f'{self.export_path}/final_summary_{rospy.Time.now().secs}.txt'
        
        with open(summary_file, 'w') as f:
            f.write("=== exploration performance summary ===\n\n")
            
            total_time = (rospy.Time.now() - self.start_time).to_sec()
            f.write(f"total exploration time: {total_time:.2f} seconds\n")
            f.write(f"final explored area: {self.explored_area:.2f}%\n")
            f.write(f"final frontier count: {len(self.frontiers)}\n\n")
            
            f.write("=== robot performance ===\n")
            for robot in self.robot_names:
                f.write(f"{robot}:\n")
                f.write(f"  total distance: {self.total_distance[robot]:.2f}m\n")
                f.write(f"  average speed: {self.total_distance[robot]/total_time:.3f}m/s\n")
            f.write("\n")
            
            f.write("=== average metrics ===\n")
            for metric_name, values in self.metrics.items():
                if values:
                    avg_value = np.mean(list(values))
                    f.write(f"{metric_name}: {avg_value:.4f}\n")
            
            f.write(f"\nefficiency score: {self.explored_area / max(sum(self.total_distance.values()), 1.0):.4f} %/m\n")
    
    def setup_plot(self):
        """setup live plotting (optional)"""
        plt.ion()
        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('real-time exploration metrics')
        
        # placeholder for live plotting functionality
    
    def __del__(self):
        """cleanup"""
        if hasattr(self, 'metrics_file'):
            self.metrics_file.close()
        
        for file_obj in self.robot_data_files.values():
            file_obj.close()

if __name__ == '__main__':
    try:
        tracker = Tracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass