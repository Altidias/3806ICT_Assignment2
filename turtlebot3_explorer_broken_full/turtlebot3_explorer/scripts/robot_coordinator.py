#!/usr/bin/env python3

import rospy
rospy.loginfo("=== ROBOT COORDINATOR SCRIPT STARTED ===")
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
                                    SelectFrontiers, SelectFrontiersRequest)

class RobotState(Enum):
    IDLE = "idle"
    MOVING = "moving"
    RETURNING = "returning"
    DONE = "done"

class Robot:
    def __init__(self, id):
        self.id = id
        self.state = RobotState.IDLE
        self.pose = None
        self.target = None
        self.active = False
        self.reason = ""
        
class Coordinator:
    def __init__(self):
        rospy.init_node('robot_coordinator')
        
        # params
        self.names = rospy.get_param('~robot_names', ['robot1', 'robot2'])
        self.rate = rospy.get_param('~update_rate', 2.0)
        
        # state
        self.robots = {}
        self.map = None
        self.frontiers = []
        self.lock = Lock()
        self.all_found = False
        self.complete = False
        
        # init robots
        for n in self.names:
            self.robots[n] = Robot(n)
        
        rospy.loginfo("waiting for astar planner...")
        try:
            rospy.wait_for_service('/astar_planner/plan_path', timeout=10.0)
            self.planner = rospy.ServiceProxy('/astar_planner/plan_path', PlanPath)
            rospy.loginfo("✓ astar planner connected")
        except:
            rospy.logerr("failed to connect to astar planner!")
            
        # llm service
        rospy.loginfo("waiting for llm frontier selector...")
        try:
            rospy.wait_for_service('/llm_select_frontiers', timeout=10.0)
            self.llm_srv = rospy.ServiceProxy('/llm_select_frontiers', SelectFrontiers)
            rospy.loginfo("✓ llm service connected")
        except:
            rospy.logerr("failed to connect to llm service!")
        
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
        
        # timer
        self.timer = rospy.Timer(rospy.Duration(1.0/self.rate), self._update)

    def _map_cb(self, msg):
        with self.lock:
            self.map = msg
    
    def _front_cb(self, msg):
        with self.lock:
            self.frontiers = [(p.position.x, p.position.y) for p in msg.poses]
    
    def _found_cb(self, msg):
        with self.lock:
            if msg.data and not self.all_found:
                self.all_found = True
                # set all to return
                for r in self.robots.values():
                    if r.state != RobotState.DONE:
                        r.state = RobotState.RETURNING
    
    def _odom_cb(self, msg, rid):
        with self.lock:
            if rid in self.robots:
                self.robots[rid].pose = msg.pose.pose
    
    def _done_cb(self, msg, rid):
        with self.lock:
            if rid in self.robots and msg.data:
                r = self.robots[rid]
                r.active = False
                
                if r.state == RobotState.RETURNING:
                    # check if home
                    if r.pose:
                        d = robot_utils.euclidean_distance(
                            0.0, 0.0, r.pose.position.x, r.pose.position.y
                        )
                        if d < 1.0:
                            r.state = RobotState.DONE
                else:
                    # back to idle
                    r.state = RobotState.IDLE
                    r.target = None
                    r.reason = ""
                    
                    # immediately assign new frontier instead of waiting
                    if not self.all_found:
                        self._assign_single_robot(r)

    def _update(self, event):
        """main loop"""
        with self.lock:
            rospy.loginfo_throttle(5.0, f"Update: frontiers={len(self.frontiers)}, robots={[(r.id, r.state.value) for r in self.robots.values()]}")
            
            # check done
            if self._check_done():
                return
            
            # assign tasks or handle returns
            if not self.all_found:
                # Check for idle robots that need assignments
                idle_robots = [r for r in self.robots.values() 
                            if r.state == RobotState.IDLE and not r.active and r.pose]
                
                if idle_robots and self.frontiers:
                    rospy.loginfo(f"Found {len(idle_robots)} idle robots, assigning frontiers...")
                    for robot in idle_robots:
                        self._assign_single_robot(robot)
            else:
                self._return_home()
            
            # publish status/viz
            self._pub_status()
            self._pub_viz()

    def _assign_single_robot(self, robot):
        """assign frontier to single robot"""
        if not self.frontiers or not self.map or not robot.pose:
            return
        
        # get assigned frontiers
        assigned = []
        for i, f in enumerate(self.frontiers):
            for r in self.robots.values():
                if (r.target and r.state == RobotState.MOVING and
                    abs(r.target[0] - f[0]) < 0.1 and
                    abs(r.target[1] - f[1]) < 0.1):
                    assigned.append(i)
                    break
        
        # call llm for just this robot
        req = SelectFrontiersRequest()
        req.robot_ids = [robot.id]
        
        # robot pose
        poses = PoseArray()
        p = Pose()
        p.position.x = robot.pose.position.x
        p.position.y = robot.pose.position.y
        p.orientation = robot.pose.orientation
        poses.poses.append(p)
        req.robot_poses = poses
        
        # frontiers
        fronts = PoseArray()
        for fx, fy in self.frontiers:
            p = Pose()
            p.position.x = fx
            p.position.y = fy
            p.orientation.w = 1.0
            fronts.poses.append(p)
        req.frontiers = fronts
        
        req.assigned_indices = assigned
        
        try:
            # call llm
            resp = self.llm_srv(req)
            
            if resp.success and resp.frontier_indices:
                idx = resp.frontier_indices[0]
                
                if 0 <= idx < len(self.frontiers):
                    f = self.frontiers[idx]
                    robot.target = f
                    robot.reason = resp.reasoning[0] if resp.reasoning else ""
                    
                    # send robot
                    if not self._send_to(robot, f[0], f[1]):
                        robot.target = None
                        robot.reason = ""
                        
        except Exception as e:
            rospy.logerr(f"llm fail: {e}")

    def _check_done(self):
        """check if complete"""
        if not self.all_found:
            return False
        
        all_done = all(r.state == RobotState.DONE for r in self.robots.values())
        
        if all_done and not self.complete:
            self.complete = True
            msg = Bool()
            msg.data = True
            self.done_pub.publish(msg)
        
        return self.complete

    def _return_home(self):
        """send robots home"""
        for r in self.robots.values():
            if r.state == RobotState.RETURNING and not r.active and r.pose:
                self._send_to(r, 0.0, 0.0)

    def _assign_llm(self):
        """llm frontier assignment"""
        if not self.frontiers or not self.map:
            return
        
        # get idle
        idle = [r for r in self.robots.values() 
                if r.state == RobotState.IDLE and not r.active and r.pose]
        
        if not idle:
            return
        
        # get assigned
        assigned = []
        for i, f in enumerate(self.frontiers):
            for r in self.robots.values():
                if (r.target and r.state == RobotState.MOVING and
                    abs(r.target[0] - f[0]) < 0.1 and
                    abs(r.target[1] - f[1]) < 0.1):
                    assigned.append(i)
                    break
        
        # build request
        req = SelectFrontiersRequest()
        req.robot_ids = [r.id for r in idle]
        
        # poses
        poses = PoseArray()
        for r in idle:
            p = Pose()
            p.position.x = r.pose.position.x
            p.position.y = r.pose.position.y
            p.orientation = r.pose.orientation
            poses.poses.append(p)
        req.robot_poses = poses
        
        # frontiers
        fronts = PoseArray()
        for fx, fy in self.frontiers:
            p = Pose()
            p.position.x = fx
            p.position.y = fy
            p.orientation.w = 1.0
            fronts.poses.append(p)
        req.frontiers = fronts
        
        req.assigned_indices = assigned
        
        try:
            # call llm
            resp = self.llm_srv(req)
            
            if resp.success:
                # process
                for i, r in enumerate(idle):
                    if i < len(resp.frontier_indices):
                        idx = resp.frontier_indices[i]
                        
                        if 0 <= idx < len(self.frontiers):
                            f = self.frontiers[idx]
                            r.target = f
                            r.reason = resp.reasoning[i] if i < len(resp.reasoning) else ""
                            
                            # send robot
                            if not self._send_to(r, f[0], f[1]):
                                r.target = None
                                r.reason = ""
                
        except Exception as e:
            rospy.logerr(f"llm fail: {e}")

    def _send_to(self, robot, gx, gy):
        """send robot to goal"""
        if not self.map or not robot.pose:
            return False
        
        try:
            # plan
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
            
            resp = self.planner(req)
            
            if not resp.success or not resp.path_x:
                return False
            
            # follow
            freq = FollowPathRequest()
            freq.path_x = [robot_utils.map_to_world(x, y, self.map.info)[0] 
                          for x, y in zip(resp.path_x, resp.path_y)]
            freq.path_y = [robot_utils.map_to_world(x, y, self.map.info)[1] 
                          for x, y in zip(resp.path_x, resp.path_y)]
            
            fresp = self.path_srvs[robot.id](freq)
            
            if fresp.success:
                robot.state = RobotState.MOVING
                robot.active = True
                return True
                
        except Exception:
            pass
        
        return False

    def _pub_status(self):
        """pub status"""
        lines = []
        
        # robots
        for r in self.robots.values():
            s = f"{r.id}: {r.state.value}"
            if r.reason:
                s += f" ({r.reason})"
            lines.append(s)
        
        lines.append(f"frontiers: {len(self.frontiers)}")
        
        if self.all_found:
            lines.append("returning")
        elif self.complete:
            lines.append("complete")
        
        msg = String()
        msg.data = "\n".join(lines)
        self.status_pub.publish(msg)

    def _pub_viz(self):
        """viz assignments"""
        markers = MarkerArray()
        
        # clear
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        
        # lines
        mid = 0
        for r in self.robots.values():
            if r.target and r.pose and r.state == RobotState.MOVING:
                # line
                m = Marker()
                m.header.frame_id = "map"
                m.header.stamp = rospy.Time.now()
                m.ns = "assignments"
                m.id = mid
                m.type = Marker.LINE_STRIP
                m.action = Marker.ADD
                
                # points
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
                
                # color
                if r.id == "robot1":
                    m.color.r = 1.0
                else:
                    m.color.g = 1.0
                
                markers.markers.append(m)
                mid += 1
        
        self.viz_pub.publish(markers)

if __name__ == '__main__':
    try:
        coordinator = Coordinator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass