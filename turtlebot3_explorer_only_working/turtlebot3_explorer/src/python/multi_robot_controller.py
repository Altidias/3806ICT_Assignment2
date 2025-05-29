#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from gazebo_msgs.srv import GetModelState
from turtlebot3_explorer.msg import ExplorationPlan, ExecutionStatus, AgentState
from astar_turtlebot3_nav.srv import PlanPath, PlanPathRequest, GetPathCosts, GetPathCostsRequest
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
import tf2_ros
import tf2_geometry_msgs
import threading
from enum import Enum
import math
import time

class RobotState(Enum):
    IDLE = "idle"
    PLANNING = "planning"
    FOLLOWING_PATH = "following_path"
    ROTATING = "rotating"
    STUCK = "stuck"
    REACHED_GOAL = "reached_goal"

class PathFollower:
    """Handles path following using velocity commands"""
    def __init__(self, robot_id, cmd_vel_pub):
        self.robot_id = robot_id
        self.cmd_vel_pub = cmd_vel_pub
        self.current_path = []
        self.path_index = 0
        self.position_tolerance = 0.2  # meters
        self.angular_tolerance = 0.2   # radians
        self.linear_speed = 0.3        # m/s
        self.angular_speed = 0.4       # rad/s
        
    def set_path(self, path_x, path_y):
        """Set a new path to follow"""
        self.current_path = [(path_x[i], path_y[i]) for i in range(len(path_x))]
        self.path_index = 0
        
    def get_target_point(self):
        """Get the current target point"""
        if self.path_index < len(self.current_path):
            return self.current_path[self.path_index]
        return None
        
    def update(self, current_pose):
        """Update path following and return velocity command"""
        cmd = Twist()
        
        if self.path_index >= len(self.current_path):
            return cmd, True  # Path complete
            
        target = self.current_path[self.path_index]
        
        # Calculate distance and angle to target
        dx = target[0] - current_pose.position.x
        dy = target[1] - current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate current robot yaw
        current_yaw = self.get_yaw_from_quaternion(current_pose.orientation)
        
        # Calculate desired angle
        desired_yaw = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(desired_yaw - current_yaw)
        
        # Check if we reached the current waypoint
        if distance < self.position_tolerance:
            self.path_index += 1
            rospy.loginfo(f"{self.robot_id}: Reached waypoint {self.path_index}/{len(self.current_path)}")
            return cmd, self.path_index >= len(self.current_path)
        
        # Rotate first if needed
        if abs(angle_diff) > self.angular_tolerance:
            cmd.angular.z = self.angular_speed * np.sign(angle_diff)
            cmd.linear.x = 0.0
        else:
            # Move forward with reduced speed if close to waypoint
            if distance < self.position_tolerance * 2:
                cmd.linear.x = self.linear_speed * 0.5  # Slow down near waypoint
            else:
                cmd.linear.x = min(self.linear_speed, distance)
            # Small angular correction while moving
            cmd.angular.z = angle_diff * 0.5
            
        return cmd, False
        
    def get_yaw_from_quaternion(self, q):
        """Extract yaw from quaternion"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

class RobotInfo:
    def __init__(self, robot_id, robot_type="explorer"):
        self.id = robot_id
        self.type = robot_type
        self.state = RobotState.IDLE
        self.current_pose = None
        self.target_frontier = None
        self.exploration_history = []
        self.path_follower = None
        self.last_update_time = rospy.Time.now()
        self.stuck_counter = 0
        self.last_position = None
        
class MultiRobotController:
    def __init__(self):
        rospy.init_node('multi_robot_controller')
        
        # Robot management
        self.robots = {}
        self.robot_namespaces = rospy.get_param('~robot_namespaces', ['robot1', 'robot2'])
        
        # Map data
        self.current_map = None
        self.map_lock = threading.Lock()
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Initialize robots
        for ns in self.robot_namespaces:
            self.robots[ns] = RobotInfo(ns, "explorer")
            # Create velocity command publisher
            cmd_vel_pub = rospy.Publisher(f'/{ns}/cmd_vel', Twist, queue_size=1)
            self.robots[ns].path_follower = PathFollower(ns, cmd_vel_pub)
        
        # Service clients for A* planner
        rospy.wait_for_service('plan_path', timeout=10.0)
        rospy.wait_for_service('get_path_costs', timeout=10.0)
        self.plan_path_client = rospy.ServiceProxy('plan_path', PlanPath)
        self.get_costs_client = rospy.ServiceProxy('get_path_costs', GetPathCosts)
        
        # Publishers
        self.plan_pub = rospy.Publisher('/exploration_plan', ExplorationPlan, queue_size=1)
        self.status_pubs = {}
        
        for robot_id in self.robots:
            self.status_pubs[robot_id] = rospy.Publisher(
                f'/{robot_id}/execution_status', ExecutionStatus, queue_size=1
            )
        
        # Subscribers
        self.frontiers_sub = rospy.Subscriber('/frontiers', PoseArray, self.frontiers_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        # Odometry subscribers for each robot
        self.odom_subs = {}
        for robot_id in self.robots:
            self.odom_subs[robot_id] = rospy.Subscriber(
                f'/{robot_id}/odom', Odometry, 
                lambda msg, rid=robot_id: self.odom_callback(msg, rid)
            )
        
        self.current_frontiers = []
        self.lock = threading.Lock()
        
        # Control loop
        self.control_rate = rospy.Rate(10)  # 10 Hz
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        rospy.loginfo("Multi-Robot Controller initialized with A* planner integration")
        
        
    def map_callback(self, msg):
        """Store the current map"""
        with self.map_lock:
            self.current_map = msg
            
    def odom_callback(self, msg, robot_id):
        """Update robot pose from odometry"""
        if robot_id in self.robots:
            self.robots[robot_id].current_pose = msg.pose.pose
            
    def frontiers_callback(self, msg):
        """Store current frontiers"""
        with self.lock:
            self.current_frontiers = [(p.position.x, p.position.y) for p in msg.poses]
            rospy.loginfo_throttle(5.0, f"Received {len(self.current_frontiers)} frontiers")
            
    def get_robot_pose_in_map_frame(self, robot_id):
        """Get robot pose transformed to map frame"""
        try:
            # Get transform from robot's odom frame to map frame
            transform = self.tf_buffer.lookup_transform(
                'map', f'{robot_id}/odom', rospy.Time(0), rospy.Duration(1.0)
            )
            
            # Transform the robot's pose
            if self.robots[robot_id].current_pose:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = f'{robot_id}/odom'
                pose_stamped.pose = self.robots[robot_id].current_pose
                
                transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
                return transformed_pose.pose
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"TF error for {robot_id}: {e}")
            
        return self.robots[robot_id].current_pose
        
    def world_to_map_coords(self, world_x, world_y):
        """Convert world coordinates to map grid coordinates"""
        if not self.current_map:
            return None, None
            
        map_x = int((world_x - self.current_map.info.origin.position.x) / self.current_map.info.resolution)
        map_y = int((world_y - self.current_map.info.origin.position.y) / self.current_map.info.resolution)
        
        return map_x, map_y
        
    def map_to_world_coords(self, map_x, map_y):
        """Convert map grid coordinates to world coordinates"""
        if not self.current_map:
            return None, None
            
        world_x = map_x * self.current_map.info.resolution + self.current_map.info.origin.position.x
        world_y = map_y * self.current_map.info.resolution + self.current_map.info.origin.position.y
        
        return world_x, world_y
        
    def convert_occupancy_grid_for_astar(self, occupancy_grid, goal_mx=None, goal_my=None):
        """Convert ROS OccupancyGrid to A* planner format"""
        converted_data = []
        for i, value in enumerate(occupancy_grid.data):
            # Calculate x,y from index
            y = i // occupancy_grid.info.width
            x = i % occupancy_grid.info.width
            
            if value == -1:
                # If we have a goal and this unknown cell is near it, treat as free
                if goal_mx is not None and goal_my is not None:
                    dist_to_goal = abs(x - goal_mx) + abs(y - goal_my)
                    if dist_to_goal <= 10:  # Within 5 cells of goal
                        converted_data.append(0)  # Treat as free
                    else:
                        converted_data.append(-1)  # Keep as unknown
                else:
                    converted_data.append(-1)  # Unknown
            elif value < 50:
                converted_data.append(0)   # Free
            else:
                converted_data.append(1)   # Occupied
        return converted_data
        
    def plan_path_for_robot(self, robot_id, goal_x, goal_y):
        """Plan a path using the A* service"""
        if not self.current_map:
            rospy.logwarn(f"No map available for path planning")
            return None
            
        robot_pose = self.get_robot_pose_in_map_frame(robot_id)
        if not robot_pose:
            rospy.logwarn(f"No pose available for {robot_id}")
            return None
            
        # Convert positions to map coordinates
        start_mx, start_my = self.world_to_map_coords(
            robot_pose.position.x, robot_pose.position.y
        )
        goal_mx, goal_my = self.world_to_map_coords(goal_x, goal_y)
        
        if None in [start_mx, start_my, goal_mx, goal_my]:
            rospy.logwarn(f"Invalid coordinates for path planning")
            return None
            
        # Fix robot seeing itself as obstacle
        if self.current_map:
            if start_mx is not None:
                idx = start_my * self.current_map.info.width + start_mx  
                if 0 <= idx < len(self.current_map.data) and self.current_map.data[idx] != 0:
                    for d in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,-1), (1,-1), (-1,1)]:
                        nx, ny = start_mx + d[0], start_my + d[1]
                        if 0 <= nx < self.current_map.info.width and 0 <= ny < self.current_map.info.height:
                            nidx = ny * self.current_map.info.width + nx
                            if self.current_map.data[nidx] == 0:
                                rospy.loginfo(f"Fixed start: ({start_mx},{start_my}) -> ({nx},{ny})")
                                start_mx, start_my = nx, ny
                                break
                                
        # Fix frontier goals in unknown space
        if self.current_map and goal_mx is not None:
            idx = goal_my * self.current_map.info.width + goal_mx
            if 0 <= idx < len(self.current_map.data) and self.current_map.data[idx] != 0:
                for r in range(1, 20):
                    found = False
                    for dx in range(-r, r+1):
                        for dy in range(-r, r+1):
                            nx, ny = goal_mx + dx, goal_my + dy
                            if 0 <= nx < self.current_map.info.width and 0 <= ny < self.current_map.info.height:
                                nidx = ny * self.current_map.info.width + nx
                                if self.current_map.data[nidx] == 0:
                                    rospy.loginfo(f"Fixed goal: ({goal_mx},{goal_my}) -> ({nx},{ny})")
                                    goal_mx, goal_my = nx, ny
                                    found = True
                                    break
                        if found: break
                    if found: break
        
            
        # Prepare service request
        try:
            req = PlanPathRequest()
            req.map_data = self.convert_occupancy_grid_for_astar(self.current_map, goal_mx, goal_my)
            req.map_width = self.current_map.info.width
            req.map_height = self.current_map.info.height
            req.start_x = start_mx
            req.start_y = start_my
            req.goal_x = goal_mx
            req.goal_y = goal_my
            req.robot_id = robot_id
            
            # Call the service
            resp = self.plan_path_client(req)
            
            if resp.success and len(resp.path_x) > 0:
                # Convert path back to world coordinates
                world_path_x = []
                world_path_y = []
                for i in range(len(resp.path_x)):
                    wx, wy = self.map_to_world_coords(resp.path_x[i], resp.path_y[i])
                    if wx is not None and wy is not None:
                        world_path_x.append(wx)
                        world_path_y.append(wy)
                        
                return (world_path_x, world_path_y)
            else:
                rospy.logwarn(f"Path planning failed for {robot_id}")
                return None
                
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None
            
    def calculate_frontier_utility(self, robot_pose, frontier, robot_info):
        """Calculate utility score for frontier assignment"""
        # Distance cost
        distance = math.sqrt(
            (frontier[0] - robot_pose.position.x)**2 + 
            (frontier[1] - robot_pose.position.y)**2
        )
        
        # Base utility
        utility = -distance  # Negative because closer is better
        
        # Penalty for recently visited
        if frontier in robot_info.exploration_history[-5:]:
            utility -= 10.0
            
        return utility
        
    def assign_frontiers_to_explorers(self):
        """Assign frontiers to idle robots"""
        if not self.current_frontiers:
            return
            
        available_frontiers = self.current_frontiers.copy()
        assignments = {}
        
        for robot in self.robots.values():
            if robot.state != RobotState.IDLE:
                continue
                
            robot_pose = self.get_robot_pose_in_map_frame(robot.id)
            if not robot_pose:
                continue
                
            # Find best frontier
            best_frontier = None
            best_utility = float('-inf')
            
            for frontier in available_frontiers:
                utility = self.calculate_frontier_utility(robot_pose, frontier, robot)
                if utility > best_utility:
                    best_utility = utility
                    best_frontier = frontier
                    
            if best_frontier:
                assignments[robot.id] = best_frontier
                available_frontiers.remove(best_frontier)
                robot.target_frontier = best_frontier
                robot.state = RobotState.PLANNING
                
        return assignments
        
    def execute_robot_path(self, robot_info):
        """Execute path following for a robot"""
        if not robot_info.current_pose:
            return
            
        # Get velocity command from path follower
        cmd, path_complete = robot_info.path_follower.update(robot_info.current_pose)
        
        # Publish velocity command
        robot_info.path_follower.cmd_vel_pub.publish(cmd)
        
        # Check if path is complete
        if path_complete:
            robot_info.state = RobotState.REACHED_GOAL
            robot_info.exploration_history.append(robot_info.target_frontier)
            robot_info.target_frontier = None
            # Stop the robot
            stop_cmd = Twist()
            robot_info.path_follower.cmd_vel_pub.publish(stop_cmd)
            rospy.loginfo(f"{robot_info.id} reached goal!")
            
    def check_robot_stuck(self, robot_info):
        """Check if robot is stuck"""
        if robot_info.last_position and robot_info.current_pose:
            distance_moved = math.sqrt(
                (robot_info.current_pose.position.x - robot_info.last_position.x)**2 +
                (robot_info.current_pose.position.y - robot_info.last_position.y)**2
            )
            
            if distance_moved < 0.005:  # Less than 1cm in 1 second
                robot_info.stuck_counter += 1
                if robot_info.stuck_counter > 30:  # Stuck for 3 seconds
                    return True
            else:
                robot_info.stuck_counter = 0
                
        robot_info.last_position = robot_info.current_pose.position if robot_info.current_pose else None
        return False
        
    def control_loop(self, event):
        """Main control loop"""
        with self.lock:
            # Assign frontiers to idle robots
            assignments = self.assign_frontiers_to_explorers()
            
            # Process each robot
            for robot in self.robots.values():
                if robot.state == RobotState.PLANNING:
                    # Plan path to assigned frontier
                    if robot.target_frontier:
                        path = self.plan_path_for_robot(
                            robot.id, 
                            robot.target_frontier[0], 
                            robot.target_frontier[1]
                        )
                        
                        if path:
                            robot.path_follower.set_path(path[0], path[1])
                            robot.state = RobotState.FOLLOWING_PATH
                            rospy.loginfo(f"{robot.id} starting path with {len(path[0])} waypoints")
                        else:
                            rospy.logwarn(f"Failed to plan path for {robot.id}")
                            robot.state = RobotState.IDLE
                            robot.target_frontier = None
                            
                elif robot.state == RobotState.FOLLOWING_PATH:
                    # Execute path following
                    self.execute_robot_path(robot)
                    
                    # Check if stuck
                    if self.check_robot_stuck(robot):
                        rospy.logwarn(f"{robot.id} appears to be stuck!")
                        robot.state = RobotState.STUCK
                        # Stop the robot
                        stop_cmd = Twist()
                        robot.path_follower.cmd_vel_pub.publish(stop_cmd)
                        
                elif robot.state == RobotState.STUCK:
                    # Try to recover from being stuck
                    # Simple recovery: rotate in place
                    cmd = Twist()
                    cmd.linear.x = -0.1
                    robot.path_follower.cmd_vel_pub.publish(cmd)

                    robot.stuck_counter += 1
                    
                    # After some time, try again
                    if robot.stuck_counter > 50:
                        robot.state = RobotState.IDLE
                        robot.stuck_counter = 0
                        robot.target_frontier = None
                        
                elif robot.state == RobotState.REACHED_GOAL:
                    # Transition back to idle
                    robot.state = RobotState.IDLE
                    
        # Publish status updates
        self.publish_status_updates()
        
    def publish_status_updates(self):
        """Publish status for all robots"""
        for robot_id, robot in self.robots.items():
            status = ExecutionStatus()
            status.header.stamp = rospy.Time.now()
            status.robot_id = robot_id
            status.status = robot.state.value
            
            if robot.current_pose:
                status.current_pose = robot.current_pose
                
            if robot.target_frontier:
                status.target_pose.position.x = robot.target_frontier[0]
                status.target_pose.position.y = robot.target_frontier[1]
                
            # Progress percentage
            if robot.state == RobotState.FOLLOWING_PATH and robot.path_follower.current_path:
                progress = (robot.path_follower.path_index / len(robot.path_follower.current_path)) * 100
                status.progress_percentage = progress
                
            self.status_pubs[robot_id].publish(status)

if __name__ == '__main__':
    try:
        controller = MultiRobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass