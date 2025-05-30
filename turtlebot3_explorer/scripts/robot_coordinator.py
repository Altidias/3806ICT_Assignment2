#!/usr/bin/env python3

import rospy
import math
import sys
import os
from enum import Enum
from threading import Lock

# hack to import
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
import robot_utils

from geometry_msgs.msg import PoseArray, Point, Pose
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray, Marker
from turtlebot3_explorer.srv import (PlanPath, PlanPathRequest, 
                                    FollowPath, FollowPathRequest,
                                    SelectFrontiers, SelectFrontiersRequest,
                                    OptimizePath, OptimizePathRequest)

class RobotState(Enum):
    IDLE = "idle"
    MOVING = "moving"
    RETURNING = "returning"
    DONE = "done"
    RESCUE = "rescue"
    FAILED = "failed"

class Robot:
    def __init__(self, id):
        self.id = id
        self.state = RobotState.IDLE
        self.pose = None
        self.target = None
        self.active = False
        self.reason = ""
        self.rescue_targets = []
        self.current_target_idx = 0
        self.failed_attempts = 0
        self.last_move_time = rospy.Time.now()
        self.stuck_counter = 0
        self.home_x = 0.0
        self.home_y = 0.0

class Coordinator:
    def __init__(self):
        rospy.init_node('robot_coordinator')
        
        # params
        self.names = rospy.get_param('~robot_names', ['robot1', 'robot2'])
        self.rate = rospy.get_param('~update_rate', 2.0)
        self.max_failed_attempts = rospy.get_param('~max_failed_attempts', 3)
        self.stuck_timeout = rospy.get_param('~stuck_timeout', 30.0)
        self.min_frontier_distance = rospy.get_param('~min_frontier_distance', 0.5)
        self.home_tolerance = rospy.get_param('~home_tolerance', 1.0)
        
        # state
        self.robots = {}
        self.map = None
        self.frontiers = []
        self.lock = Lock()
        self.all_found = False
        self.rescue_mode = False
        self.complete = False
        self.last_frontier_time = rospy.Time.now()
        
        # init robots with spawn positions
        for n in self.names:
            robot = Robot(n)
            robot.home_x = rospy.get_param(f'/{n}_spawn_x', 0.0)
            robot.home_y = rospy.get_param(f'/{n}_spawn_y', 0.0)
            self.robots[n] = robot
        
        # services
        try:
            rospy.wait_for_service('/astar_planner/plan_path', timeout=10.0)
            self.planner = rospy.ServiceProxy('/astar_planner/plan_path', PlanPath)
        except:
            rospy.logerr("failed to connect to astar planner!")
            
        # llm services
        try:
            rospy.wait_for_service('/llm_select_frontiers', timeout=10.0)
            self.llm_frontier_srv = rospy.ServiceProxy('/llm_select_frontiers', SelectFrontiers)
        except:
            rospy.logerr("failed to connect to llm frontier service!")

        try:
            rospy.wait_for_service('/llm_optimize_path', timeout=10.0)
            self.llm_rescue_srv = rospy.ServiceProxy('/llm_optimize_path', OptimizePath)
        except:
            rospy.logerr("failed to connect to llm rescue service!")
        
        # path services
        self.path_srvs = {}
        for n in self.names:
            svc = f'/{n}/follow_path'
            rospy.wait_for_service(svc)
            self.path_srvs[n] = rospy.ServiceProxy(svc, FollowPath)
        
        # subs
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self._map_cb)
        self.front_sub = rospy.Subscriber('/frontiers', PoseArray, self._front_cb)
        self.found_sub = rospy.Subscriber('/all_survivors_found', Bool, self._found_cb)
        
        # robot subs
        for n in self.names:
            rospy.Subscriber(f'/{n}/odom', Odometry,
                           lambda m, r=n: self._odom_cb(m, r))
            rospy.Subscriber(f'/{n}/path_complete', Bool,
                           lambda m, r=n: self._done_cb(m, r))
        
        # pubs
        self.status_pub = rospy.Publisher('/exploration_status', String, queue_size=1, latch=True)
        self.done_pub = rospy.Publisher('/exploration_complete', Bool, queue_size=1, latch=True)
        self.viz_pub = rospy.Publisher('/frontier_assignments', MarkerArray, queue_size=1, latch=True)
        self.rescue_status_pub = rospy.Publisher('/rescue_status', String, queue_size=1, latch=True)
        self.debug_pub = rospy.Publisher('/coordinator_debug', String, queue_size=1, latch=True)
        
        # timer
        self.timer = rospy.Timer(rospy.Duration(1.0/self.rate), self._update)
        
        self._pub_status()

    def _map_cb(self, msg):
        with self.lock:
            self.map = msg
    
    def _front_cb(self, msg):
        with self.lock:
            old_count = len(self.frontiers)
            self.frontiers = [(p.position.x, p.position.y) for p in msg.poses]
            
            if len(self.frontiers) > 0:
                self.last_frontier_time = rospy.Time.now()
            
            if len(self.frontiers) > old_count:
                for r in self.robots.values():
                    r.failed_attempts = 0
    
    def _found_cb(self, msg):
        with self.lock:
            if msg.data and not self.all_found:
                self.all_found = True
                # set all active explorer robots to return home
                for r in self.robots.values():
                    if r.state in [RobotState.IDLE, RobotState.MOVING]:
                        r.state = RobotState.RETURNING
                        r.active = False
                        r.target = None
                        r.failed_attempts = 0
    
    def _odom_cb(self, msg, rid):
        with self.lock:
            if rid in self.robots:
                old_pose = self.robots[rid].pose
                self.robots[rid].pose = msg.pose.pose
                
                # check if robot is moving
                if old_pose:
                    distance_moved = robot_utils.euclidean_distance(
                        old_pose.position.x, old_pose.position.y,
                        msg.pose.pose.position.x, msg.pose.pose.position.y
                    )
                    
                    if distance_moved > 0.05:
                        self.robots[rid].last_move_time = rospy.Time.now()
                        self.robots[rid].stuck_counter = 0
                    else:
                        time_since_move = (rospy.Time.now() - self.robots[rid].last_move_time).to_sec()
                        if time_since_move > self.stuck_timeout and self.robots[rid].state in [RobotState.MOVING, RobotState.RETURNING]:
                            self.robots[rid].stuck_counter += 1
                            
                            if self.robots[rid].stuck_counter > 3:
                                self.robots[rid].state = RobotState.FAILED
                                self.robots[rid].active = False
    
    def _done_cb(self, msg, rid):
        with self.lock:
            if rid in self.robots and msg.data:
                r = self.robots[rid]
                r.active = False
                
                if r.state == RobotState.RETURNING:
                    # check if robot is now at home
                    if r.pose:
                        d = robot_utils.euclidean_distance(
                            r.home_x, r.home_y, 
                            r.pose.position.x, r.pose.position.y
                        )
                        if d < self.home_tolerance:
                            r.state = RobotState.DONE
                        else:
                            # still not at home, keep trying
                            r.state = RobotState.RETURNING
                elif r.state == RobotState.MOVING:
                    r.state = RobotState.IDLE
                    r.target = None
                    r.reason = ""
                    r.failed_attempts = 0
                    self._assign()

    def _update(self, event):
        """main loop"""
        with self.lock:
            if not self.all_found:
                # exploration mode
                self._handle_failed()
                self._assign()
                self._check_done()
            else:
                # return home mode
                self._handle_returning()
                self._check_all_home()
            
            self._pub_viz()
            self._pub_debug()
    
    def _handle_failed(self):
        """handle failed/stuck robots"""
        for r in self.robots.values():
            if r.state == RobotState.FAILED:
                if r.failed_attempts < self.max_failed_attempts:
                    r.state = RobotState.IDLE
                    r.target = None
                    r.active = False
                    r.stuck_counter = 0
                    r.last_move_time = rospy.Time.now()
                else:
                    r.state = RobotState.DONE
    
    def _handle_returning(self):
        """handle robots returning home"""
        returning_robots = [r for r in self.robots.values() 
                          if r.state == RobotState.RETURNING and not r.active and r.pose]
        
        for robot in returning_robots:
            if robot.failed_attempts >= self.max_failed_attempts:
                robot.state = RobotState.DONE
                continue
                
            # check if already at home
            distance_to_home = robot_utils.euclidean_distance(
                robot.home_x, robot.home_y,
                robot.pose.position.x, robot.pose.position.y
            )
            
            if distance_to_home < self.home_tolerance:
                robot.state = RobotState.DONE
                continue
            
            # send robot home
            if self._send_home(robot):
                pass
            else:
                robot.failed_attempts += 1

    def _send_home(self, robot):
        """send robot home"""
        if not self.map or not robot.pose:
            return False
        
        try:
            # plan path to home
            req = PlanPathRequest()
            req.robot_id = robot.id
            req.start_x, req.start_y = robot_utils.world_to_map(
                robot.pose.position.x, robot.pose.position.y, self.map.info
            )
            req.goal_x, req.goal_y = robot_utils.world_to_map(
                robot.home_x, robot.home_y, self.map.info
            )
            req.map_data = list(self.map.data)
            req.map_width = self.map.info.width
            req.map_height = self.map.info.height
            req.use_heuristics = False
            
            # bounds check
            if (req.start_x < 0 or req.start_x >= self.map.info.width or
                req.start_y < 0 or req.start_y >= self.map.info.height or
                req.goal_x < 0 or req.goal_x >= self.map.info.width or
                req.goal_y < 0 or req.goal_y >= self.map.info.height):
                return self._send_home_fallback(robot)
            
            resp = self.planner(req)
            
            if not resp.success or not resp.path_x:
                return self._send_home_fallback(robot)
            
            # follow path
            freq = FollowPathRequest()
            freq.path_x = [robot_utils.map_to_world(x, y, self.map.info)[0] 
                          for x, y in zip(resp.path_x, resp.path_y)]
            freq.path_y = [robot_utils.map_to_world(x, y, self.map.info)[1] 
                          for x, y in zip(resp.path_x, resp.path_y)]
            
            fresp = self.path_srvs[robot.id](freq)
            
            if fresp.success:
                robot.active = True
                robot.last_move_time = rospy.Time.now()
                robot.reason = f"returning home to ({robot.home_x:.2f}, {robot.home_y:.2f})"
                return True
                
        except Exception as e:
            rospy.logerr(f"exception in return planning for {robot.id}: {e}")
        
        return self._send_home_fallback(robot)

    def _send_home_fallback(self, robot):
        """fallback method to send robot home"""
        try:
            # simple fallback: just send home coords directly
            freq = FollowPathRequest()
            freq.path_x = [robot.home_x]
            freq.path_y = [robot.home_y]
            
            fresp = self.path_srvs[robot.id](freq)
            
            if fresp.success:
                robot.active = True
                robot.last_move_time = rospy.Time.now()
                robot.reason = f"returning home (fallback) to ({robot.home_x:.2f}, {robot.home_y:.2f})"
                return True
                
        except Exception as e:
            rospy.logerr(f"fallback failed for {robot.id}: {e}")
        
        return False

    def _check_all_home(self):
        """check if all robots returned home"""
        if not self.all_found:
            return
            
        all_home = True
        for r in self.robots.values():
            if r.state not in [RobotState.DONE, RobotState.FAILED]:
                all_home = False
                break
                
        if all_home and not self.complete:
            self.complete = True
            self.done_pub.publish(Bool(True))
            
            # final status
            status_msg = String()
            status_msg.data = "all robots returned home - ready for rescue phase"
            self.status_pub.publish(status_msg)

    def _assign(self):
        """assign frontiers using llm"""
        if not self.frontiers or not self.map or self.all_found:
            return
            
        # get idle robots (excluding failed)
        idle_robots = [r for r in self.robots.values() 
                      if r.state == RobotState.IDLE and not r.active and r.pose 
                      and r.failed_attempts < self.max_failed_attempts]
                      
        if not idle_robots:
            return
            
        # filter frontiers too close to robots
        valid_frontiers = []
        for fx, fy in self.frontiers:
            min_dist_to_robot = float('inf')
            for r in self.robots.values():
                if r.pose:
                    dist = robot_utils.euclidean_distance(fx, fy, r.pose.position.x, r.pose.position.y)
                    min_dist_to_robot = min(min_dist_to_robot, dist)
            
            if min_dist_to_robot > self.min_frontier_distance:
                valid_frontiers.append((fx, fy))
        
        if not valid_frontiers:
            time_since_frontiers = (rospy.Time.now() - self.last_frontier_time).to_sec()
            if time_since_frontiers > 30.0:
                self._check_map_complete()
            return
        
        # build request
        req = SelectFrontiersRequest()
        req.robot_ids = [r.id for r in idle_robots]
        
        poses = PoseArray()
        for r in idle_robots:
            poses.poses.append(r.pose)
        req.robot_poses = poses
        
        fronts = PoseArray()
        for fx, fy in valid_frontiers:
            p = Pose()
            p.position.x = fx
            p.position.y = fy
            p.orientation.w = 1.0
            fronts.poses.append(p)
        req.frontiers = fronts
        
        assigned = []
        for i, f in enumerate(valid_frontiers):
            for r in self.robots.values():
                if (r.target and r.state == RobotState.MOVING and
                    abs(r.target[0] - f[0]) < 0.1 and
                    abs(r.target[1] - f[1]) < 0.1):
                    assigned.append(i)
                    break
        req.assigned_indices = assigned
        
        try:
            resp = self.llm_frontier_srv(req)
            
            if resp.success:
                for i, r in enumerate(idle_robots):
                    if i < len(resp.frontier_indices):
                        idx = resp.frontier_indices[i]
                        
                        if 0 <= idx < len(valid_frontiers):
                            f = valid_frontiers[idx]
                            r.target = f
                            r.reason = resp.reasoning[i] if i < len(resp.reasoning) else ""
                            
                            if not self._send_to(r, f[0], f[1]):
                                r.target = None
                                r.reason = ""
                                r.failed_attempts += 1
                
        except Exception as e:
            rospy.logerr(f"llm frontier assignment failed: {e}")
            self._assign_closest(idle_robots, valid_frontiers)

    def _assign_closest(self, robots, frontiers):
        """fallback closest frontier assignment"""
        available_frontiers = frontiers.copy()
        
        for r in robots:
            if not available_frontiers:
                break
                
            min_dist = float('inf')
            closest_idx = 0
            for i, (fx, fy) in enumerate(available_frontiers):
                dist = robot_utils.euclidean_distance(
                    r.pose.position.x, r.pose.position.y, fx, fy
                )
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i
            
            f = available_frontiers[closest_idx]
            r.target = f
            r.reason = f"closest frontier (fallback), dist: {min_dist:.2f}m"
            
            if self._send_to(r, f[0], f[1]):
                available_frontiers.pop(closest_idx)
            else:
                r.target = None
                r.reason = ""
                r.failed_attempts += 1

    def _send_to(self, robot, gx, gy):
        """send robot to goal"""
        if not self.map or not robot.pose:
            return False
        
        try:
            req = PlanPathRequest()
            req.robot_id = robot.id
            req.start_x, req.start_y = robot_utils.world_to_map(
                robot.pose.position.x, robot.pose.position.y, self.map.info
            )
            req.goal_x, req.goal_y = robot_utils.world_to_map(gx, gy, self.map.info)
            req.map_data = list(self.map.data)
            req.map_width = self.map.info.width
            req.map_height = self.map.info.height
            req.use_heuristics = False
            
            if (req.start_x < 0 or req.start_x >= self.map.info.width or
                req.start_y < 0 or req.start_y >= self.map.info.height or
                req.goal_x < 0 or req.goal_x >= self.map.info.width or
                req.goal_y < 0 or req.goal_y >= self.map.info.height):
                return False
            
            resp = self.planner(req)
            
            if not resp.success or not resp.path_x:
                return False
            
            freq = FollowPathRequest()
            freq.path_x = [robot_utils.map_to_world(x, y, self.map.info)[0] 
                          for x, y in zip(resp.path_x, resp.path_y)]
            freq.path_y = [robot_utils.map_to_world(x, y, self.map.info)[1] 
                          for x, y in zip(resp.path_x, resp.path_y)]
            
            fresp = self.path_srvs[robot.id](freq)
            
            if fresp.success:
                robot.state = RobotState.MOVING
                robot.active = True
                robot.last_move_time = rospy.Time.now()
                return True
                
        except Exception as e:
            rospy.logerr(f"exception in path planning for {robot.id}: {e}")
        
        return False

    def _check_map_complete(self):
        """check if exploration complete by map analysis"""
        if not self.map:
            return
            
        unknown_cells = sum(1 for cell in self.map.data if cell == -1)
        total_cells = len(self.map.data)
        unknown_percentage = (unknown_cells / total_cells) * 100.0
        
        if unknown_percentage < 5.0:
            for r in self.robots.values():
                if r.state == RobotState.IDLE:
                    r.state = RobotState.DONE
    
    def _check_done(self):
        """check if exploration complete"""
        pass

    def _pub_status(self):
        """publish status"""
        lines = []
        
        for r in self.robots.values():
            s = f"{r.id}: {r.state.value}"
            if r.reason:
                s += f" ({r.reason})"
            if r.failed_attempts > 0:
                s += f" [fails: {r.failed_attempts}]"
            lines.append(s)
        
        lines.append(f"frontiers: {len(self.frontiers)}")
        
        if self.all_found:
            lines.append("survivors found - robots returning home")
        elif self.complete:
            lines.append("exploration complete - ready for rescue")
        
        msg = String()
        msg.data = "\n".join(lines)
        self.status_pub.publish(msg)

    def _pub_debug(self):
        """publish debug info"""
        debug_lines = []
        debug_lines.append(f"active robots: {sum(1 for r in self.robots.values() if r.active)}")
        debug_lines.append(f"failed robots: {sum(1 for r in self.robots.values() if r.state == RobotState.FAILED)}")
        debug_lines.append(f"available frontiers: {len(self.frontiers)}")
        debug_lines.append(f"all survivors found: {self.all_found}")
        debug_lines.append(f"mission complete: {self.complete}")
        
        for r in self.robots.values():
            if r.pose:
                time_since_move = (rospy.Time.now() - r.last_move_time).to_sec()
                distance_to_home = robot_utils.euclidean_distance(
                    r.home_x, r.home_y, r.pose.position.x, r.pose.position.y
                )
                debug_lines.append(f"{r.id}: last moved {time_since_move:.1f}s ago, distance to home: {distance_to_home:.2f}m")
        
        msg = String()
        msg.data = "\n".join(debug_lines)
        self.debug_pub.publish(msg)

    def _pub_viz(self):
        """publish assignment visualization"""
        markers = MarkerArray()
        
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        
        mid = 0
        for r in self.robots.values():
            if r.pose:
                # show assignment lines for moving robots
                if r.target and r.state == RobotState.MOVING:
                    m = Marker()
                    m.header.frame_id = "map"
                    m.header.stamp = rospy.Time.now()
                    m.ns = "assignments"
                    m.id = mid
                    m.type = Marker.LINE_STRIP
                    m.action = Marker.ADD
                    
                    p1 = Point()
                    p1.x = r.pose.position.x
                    p1.y = r.pose.position.y
                    p1.z = 0.1
                    m.points.append(p1)
                    
                    p2 = Point()
                    p2.x = r.target[0]
                    p2.y = r.target[1]
                    p2.z = 0.1
                    m.points.append(p2)
                    
                    m.scale.x = 0.05
                    m.color.a = 0.5
                    
                    if r.failed_attempts > 0:
                        m.color.r = 1.0
                        m.color.g = 0.5
                    elif r.id == "robot1":
                        m.color.r = 1.0
                    else:
                        m.color.g = 1.0
                    
                    markers.markers.append(m)
                    mid += 1
                
                # show home lines for returning robots
                elif r.state == RobotState.RETURNING:
                    m = Marker()
                    m.header.frame_id = "map"
                    m.header.stamp = rospy.Time.now()
                    m.ns = "return_home"
                    m.id = mid
                    m.type = Marker.LINE_STRIP
                    m.action = Marker.ADD
                    
                    p1 = Point()
                    p1.x = r.pose.position.x
                    p1.y = r.pose.position.y
                    p1.z = 0.1
                    m.points.append(p1)
                    
                    p2 = Point()
                    p2.x = r.home_x
                    p2.y = r.home_y
                    p2.z = 0.1
                    m.points.append(p2)
                    
                    m.scale.x = 0.08
                    m.color.r = 0.0
                    m.color.g = 0.0
                    m.color.b = 1.0  # blue for return home
                    m.color.a = 0.7
                    
                    markers.markers.append(m)
                    mid += 1
        
        self.viz_pub.publish(markers)

if __name__ == '__main__':
    try:
        coordinator = Coordinator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass