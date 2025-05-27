#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from turtlebot3_explorer.msg import ExplorationPlan, ExecutionStatus, AgentState
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
import json
import threading
from enum import Enum
import math
import time

class RobotState(Enum):
    IDLE = "idle"
    EXPLORING = "exploring"
    MOVING_TO_FRONTIER = "moving_to_frontier"
    RESCUING = "rescuing"
    RETURNING = "returning"
    TELEPORTING = "teleporting"  # for teleportation

class RobotInfo:
    def __init__(self, robot_id, robot_type="explorer"):
        self.id = robot_id
        self.type = robot_type  # explorer or rescuer
        self.state = RobotState.IDLE
        self.current_pose = None
        self.target_pose = None
        self.assigned_frontier = None
        self.exploration_history = []
        self.path_waypoints = []
        self.current_waypoint_idx = 0
        self.last_teleport_time = 0
        self.teleport_cooldown = 0.5  # seconds between teleports
        
class MultiRobotController:
    def __init__(self):
        rospy.init_node('multi_robot_controller')
        
        # robot management
        self.robots = {}
        self.robot_namespaces = rospy.get_param('~robot_namespaces', ['robot1', 'robot2'])
        self.rescuer_namespace = rospy.get_param('~rescuer_namespace', None)
        
        for ns in self.robot_namespaces:
            self.robots[ns] = RobotInfo(ns, "explorer")
            
        if self.rescuer_namespace:
            self.robots[self.rescuer_namespace] = RobotInfo(self.rescuer_namespace, "rescuer")
        
        self.exploration_mode = True
        self.rescue_target = None
        
        # teleportation params
        self.teleport_step_size = rospy.get_param('~teleport_step_size', 0.1)  # smaller steps
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.3)
        self.teleport_delay = rospy.get_param('~teleport_delay', 0.5)
        self.scan_pause_duration = rospy.get_param('~scan_pause_duration', 0.3)
        
        # rl params
        self.exploration_reward_discount = 0.9
        self.frontier_assignment_weights = {
            'distance': -0.4,
            'frontier_size': 0.3,
            'unexplored_ratio': 0.3
        }
        
        # gazebo services
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/get_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        
        # publishers
        self.plan_pub = rospy.Publisher('/exploration_plan', ExplorationPlan, queue_size=1)
        self.status_pubs = {}
        self.scan_pause_pubs = {}
        
        for robot_id in self.robots:
            self.status_pubs[robot_id] = rospy.Publisher(
                f'/{robot_id}/execution_status', ExecutionStatus, queue_size=1
            )
            # pause laser scan processing
            self.scan_pause_pubs[robot_id] = rospy.Publisher(
                f'/{robot_id}/pause_scan_processing', Bool, queue_size=1
            )
        
        # subscribers
        self.frontiers_sub = rospy.Subscriber('/frontiers', PoseArray, self.frontiers_callback)
        self.mode_sub = rospy.Subscriber('/control_mode', String, self.mode_callback)
        self.rescue_target_sub = rospy.Subscriber('/rescue_target', PoseStamped, self.rescue_target_callback)
        
        # scan monitoring
        self.last_scan_times = {}
        for robot_id in self.robots:
            rospy.Subscriber(f'/{robot_id}/scan', LaserScan, 
                           lambda msg, rid=robot_id: self.scan_callback(msg, rid))
        
        self.current_frontiers = []
        self.lock = threading.Lock()
        
        # control loop
        self.control_rate = rospy.Rate(10)  # higher rate for smoother movement
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        rospy.loginfo("Multi-Robot Controller initialized with robots: %s", list(self.robots.keys()))
        rospy.loginfo("Teleport step size: %.2f, delay: %.2f", self.teleport_step_size, self.teleport_delay)
        
    def scan_callback(self, msg, robot_id):
        # monitor scan timing
        self.last_scan_times[robot_id] = rospy.Time.now()
        
    def frontiers_callback(self, msg):
        with self.lock:
            self.current_frontiers = [(p.position.x, p.position.y) for p in msg.poses]
            rospy.loginfo_throttle(10.0, f"Received {len(self.current_frontiers)} frontiers")
            
    def mode_callback(self, msg):
        if msg.data == "explore":
            self.exploration_mode = True
            rospy.loginfo("Switched to exploration mode")
        elif msg.data == "rescue":
            self.exploration_mode = False
            rospy.loginfo("Switched to rescue mode")
            
    def rescue_target_callback(self, msg):
        self.rescue_target = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo("Received rescue target: %s", self.rescue_target)
        
    def get_robot_pose(self, robot_id):
        # get pose from gazebo
        try:
            if robot_id.startswith('robot'):
                model_name = f"turtlebot3_{robot_id}"
            else:
                model_name = robot_id
            
            state = self.get_model_state(model_name, "world")
            if state.success:
                return state.pose
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to get robot pose for %s: %s", robot_id, e)
        return None
        
    def pause_scan_processing(self, robot_id, pause):
        # pause/resume scan processing
        msg = Bool()
        msg.data = pause
        self.scan_pause_pubs[robot_id].publish(msg)
        
    def smooth_teleport_robot(self, robot_id, target_x, target_y, target_yaw=0):
        # teleport with scan pause
        # check cooldown
        robot_info = self.robots[robot_id]
        current_time = time.time()
        if current_time - robot_info.last_teleport_time < robot_info.teleport_cooldown:
            return False
            
        # pause scans
        self.pause_scan_processing(robot_id, True)
        
        # wait for pending scans
        rospy.sleep(0.1)
        
        # do teleport
        model_state = ModelState()
        if robot_id.startswith('robot'):
            model_state.model_name = f"turtlebot3_{robot_id}"
        else:
            model_state.model_name = robot_id
            
        model_state.pose.position.x = target_x
        model_state.pose.position.y = target_y
        model_state.pose.position.z = 0
        
        # yaw to quaternion
        model_state.pose.orientation.z = math.sin(target_yaw / 2)
        model_state.pose.orientation.w = math.cos(target_yaw / 2)
        
        try:
            self.set_model_state(model_state)
            robot_info.last_teleport_time = current_time
            
            # wait to stabilize
            rospy.sleep(self.scan_pause_duration)
            
            # resume scans
            self.pause_scan_processing(robot_id, False)
            
            rospy.logdebug("Smoothly teleported %s to (%.2f, %.2f)", robot_id, target_x, target_y)
            return True
            
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to teleport robot %s: %s", robot_id, e)
            self.pause_scan_processing(robot_id, False)  # resume on error
            return False
        
    def calculate_frontier_utility(self, robot_pose, frontier, robot_info):
        # utility score for frontier assignment
        # distance cost
        distance = math.sqrt(
            (frontier[0] - robot_pose.position.x)**2 + 
            (frontier[1] - robot_pose.position.y)**2
        )
        
        # frontier value
        frontier_size = 1.0
        unexplored_ratio = 0.5
        
        # calculate utility
        utility = (
            self.frontier_assignment_weights['distance'] * distance +
            self.frontier_assignment_weights['frontier_size'] * frontier_size +
            self.frontier_assignment_weights['unexplored_ratio'] * unexplored_ratio
        )
        
        # penalty for recently visited
        if frontier in robot_info.exploration_history[-5:]:
            utility -= 1.0
            
        return utility
        
    def assign_frontiers_to_explorers(self):
        # assign frontiers to explorers
        if not self.current_frontiers:
            return
            
        explorers = [r for r in self.robots.values() if r.type == "explorer"]
        available_frontiers = self.current_frontiers.copy()
        
        assignments = {}
        
        for robot in explorers:
            if robot.state != RobotState.IDLE:
                continue
                
            robot_pose = self.get_robot_pose(robot.id)
            if not robot_pose:
                continue
                
            # find best frontier
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
                robot.assigned_frontier = best_frontier
                robot.state = RobotState.MOVING_TO_FRONTIER
                robot.target_pose = best_frontier
                
        # publish plan
        if assignments:
            plan = ExplorationPlan()
            plan.header.stamp = rospy.Time.now()
            plan.planner_id = "rl_frontier_allocator"
            plan.robot_ids = list(assignments.keys())
            plan.frontier_assignments.poses = [
                Pose(position=PoseStamped().pose.position) for _ in assignments
            ]
            for i, (robot_id, frontier) in enumerate(assignments.items()):
                plan.frontier_assignments.poses[i].position.x = frontier[0]
                plan.frontier_assignments.poses[i].position.y = frontier[1]
            plan.reasoning = f"Assigned {len(assignments)} frontiers using utility-based allocation"
            self.plan_pub.publish(plan)
            
    def move_robot_towards_target(self, robot_info):
        # move robot to target using teleportation
        current_pose = self.get_robot_pose(robot_info.id)
        if not current_pose or not robot_info.target_pose:
            return
            
        dx = robot_info.target_pose[0] - current_pose.position.x
        dy = robot_info.target_pose[1] - current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < self.goal_tolerance:
            # reached target
            robot_info.state = RobotState.IDLE
            robot_info.exploration_history.append(robot_info.target_pose)
            robot_info.target_pose = None
            rospy.loginfo("Robot %s reached target", robot_info.id)
        else:
            # smooth teleport
            step = min(self.teleport_step_size, distance)
            new_x = current_pose.position.x + (dx / distance) * step
            new_y = current_pose.position.y + (dy / distance) * step
            yaw = math.atan2(dy, dx)
            
            if self.smooth_teleport_robot(robot_info.id, new_x, new_y, yaw):
                robot_info.current_pose = (new_x, new_y)
                    
    def control_loop(self, event):
        # main loop
        with self.lock:
            if self.exploration_mode:
                # exploration
                self.assign_frontiers_to_explorers()
                
                # move explorers
                for robot in self.robots.values():
                    if robot.type == "explorer" and robot.state == RobotState.MOVING_TO_FRONTIER:
                        self.move_robot_towards_target(robot)
                        
            else:
                # rescue mode
                if self.rescuer_namespace and self.rescuer_namespace in self.robots:
                    rescuer = self.robots[self.rescuer_namespace]
                    
                    if self.rescue_target and rescuer.state == RobotState.IDLE:
                        # generate path
                        current_pose = self.get_robot_pose(rescuer.id)
                        if current_pose:
                            rescuer.path_waypoints = self.generate_rescue_path(current_pose, self.rescue_target)
                            rescuer.current_waypoint_idx = 0
                            rescuer.state = RobotState.RESCUING
                            
                    elif rescuer.state == RobotState.RESCUING and rescuer.path_waypoints:
                        # follow path
                        if rescuer.current_waypoint_idx < len(rescuer.path_waypoints):
                            rescuer.target_pose = rescuer.path_waypoints[rescuer.current_waypoint_idx]
                            self.move_robot_towards_target(rescuer)
                            
                            # check if reached waypoint
                            if rescuer.state == RobotState.IDLE:
                                rescuer.current_waypoint_idx += 1
                                rescuer.state = RobotState.RESCUING
                        else:
                            # done with path
                            rescuer.state = RobotState.RETURNING
                            rospy.loginfo("Rescue robot reached target, returning...")
                            
        # publish status
        self.publish_status_updates()
        
    def generate_rescue_path(self, start_pose, target):
        # path for rescue
        waypoints = []
        steps = 5
        for i in range(steps + 1):
            t = i / float(steps)
            x = start_pose.position.x + t * (target[0] - start_pose.position.x)
            y = start_pose.position.y + t * (target[1] - start_pose.position.y)
            waypoints.append((x, y))
        return waypoints
        
    def publish_status_updates(self):
        # status for all robots
        for robot_id, robot in self.robots.items():
            status = ExecutionStatus()
            status.header.stamp = rospy.Time.now()
            status.robot_id = robot_id
            status.status = robot.state.value
            
            current_pose = self.get_robot_pose(robot_id)
            if current_pose:
                status.current_pose = current_pose
                
            if robot.target_pose:
                status.target_pose.position.x = robot.target_pose[0]
                status.target_pose.position.y = robot.target_pose[1]
                
            # progress
            if robot.state == RobotState.RESCUING and robot.path_waypoints:
                status.progress_percentage = (robot.current_waypoint_idx / len(robot.path_waypoints)) * 100
                
            self.status_pubs[robot_id].publish(status)

if __name__ == '__main__':
    try:
        controller = MultiRobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass