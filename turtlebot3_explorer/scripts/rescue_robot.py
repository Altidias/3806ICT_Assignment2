#!/usr/bin/env python3

import rospy
import sys
import os
from enum import Enum
from threading import Lock

# add scripts
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
import robot_utils

from std_msgs.msg import Bool, String
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from turtlebot3_explorer.srv import (GetSurvivors, GetSurvivorsRequest, 
                                   OptimizePath, OptimizePathRequest,
                                   FollowPath, FollowPathRequest)

class RescueState(Enum):
    WAITING = "waiting"
    PLANNING = "planning"
    EXECUTING = "executing"
    DONE = "done"

class Rescue:
    def __init__(self):
        rospy.init_node('rescue_robot')
        
        # params
        self.name = rospy.get_param('~robot_name', 'rescue_robot')
        self.x0 = rospy.get_param('~spawn_x', 0.0)
        self.y0 = rospy.get_param('~spawn_y', 0.0)
        self.use_llm = rospy.get_param('~use_llm_optimization', True)
        self.max_planning_failures = rospy.get_param('~max_planning_failures', 3)
        
        # state
        self.state = RescueState.WAITING
        self.pose = None
        self.map = None
        self.survivors = []
        self.idx = 0
        self.lock = Lock()
        self.planning_failures = 0
        
        # wait for services
        rospy.wait_for_service('/survivor_manager/get_survivors')
        self.survivor_srv = rospy.ServiceProxy('/survivor_manager/get_survivors', GetSurvivors)
        
        if self.use_llm:
            try:
                rospy.wait_for_service('/llm_optimize_path', timeout=10.0)
                self.planner_srv = rospy.ServiceProxy('/llm_optimize_path', OptimizePath)
            except:
                self.use_llm = False
        
        rospy.wait_for_service(f'/{self.name}/follow_path')
        self.path_srv = rospy.ServiceProxy(f'/{self.name}/follow_path', FollowPath)
        
        # subs
        self.exp_sub = rospy.Subscriber('/exploration_complete', Bool, self._exp_cb)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self._map_cb)
        self.odom_sub = rospy.Subscriber(f'/{self.name}/odom', Odometry, self._odom_cb)
        self.path_sub = rospy.Subscriber(f'/{self.name}/path_complete', Bool, self._path_cb)
        
        # pubs
        self.status_pub = rospy.Publisher('/rescue_status', String, queue_size=1, latch=True)
        self.viz_pub = rospy.Publisher('/rescue_robot_marker', Marker, queue_size=1, latch=True)
        self.done_pub = rospy.Publisher('/rescue_complete', Bool, queue_size=1, latch=True)
        
        # timer
        self.timer = rospy.Timer(rospy.Duration(1.0), self._update)
        
        self._pub_status()
        
    def _exp_cb(self, msg):
        """exploration done"""
        with self.lock:
            if msg.data and self.state == RescueState.WAITING:
                self.state = RescueState.PLANNING
                self._pub_status()
    
    def _map_cb(self, msg):
        """map update"""
        with self.lock:
            self.map = msg
    
    def _odom_cb(self, msg):
        """odom update"""
        with self.lock:
            self.pose = msg.pose.pose
    
    def _path_cb(self, msg):
        """path done"""
        with self.lock:
            if msg.data and self.state == RescueState.EXECUTING:
                self.idx += 1
                self.state = RescueState.PLANNING
                self.planning_failures = 0
                self._pub_status()
    
    def _update(self, event):
        """main loop"""
        with self.lock:
            if self.state == RescueState.PLANNING:
                self._plan()
    
    def _plan(self):
        """plan next rescue"""
        if not self.map or not self.pose:
            return
            
        # get survivors
        try:
            req = GetSurvivorsRequest()
            req.only_discovered = True
            resp = self.survivor_srv(req)
            
            # build survivor list
            self.survivors = []
            for i in range(len(resp.survivor_ids)):
                survivor = Point()
                survivor.x = resp.x_positions[i]
                survivor.y = resp.y_positions[i]
                self.survivors.append(survivor)
                
        except Exception as e:
            rospy.logerr(f"failed to get survivors: {e}")
            return
            
        if not self.survivors:
            return
            
        if self.idx >= len(self.survivors):
            self.state = RescueState.DONE
            self.done_pub.publish(Bool(True))
            self._pub_status()
            return
            
        # check for too many failures
        if self.planning_failures >= self.max_planning_failures:
            self._exec_simple()
            return
            
        # try llm first, then fallback
        if self.use_llm and self._plan_llm():
            return
        else:
            self._plan_backup()

    def _plan_llm(self):
        """plan using llm optimization"""
        try:
            # create target poses
            targets = PoseArray()
            for i in range(self.idx, len(self.survivors)):
                pose = Pose()
                pose.position = self.survivors[i]
                pose.orientation.w = 1.0
                targets.poses.append(pose)
                
            if not targets.poses:
                return False
                
            # optimize path
            req = OptimizePathRequest()
            req.targets = targets
            resp = self.planner_srv(req)
            
            if not resp.optimized_targets.poses:
                return False
                
            # use first target
            target = resp.optimized_targets.poses[0]
            return self._exec(target.position.x, target.position.y)
            
        except Exception as e:
            rospy.logerr(f"llm planning failed: {e}")
            return False

    def _plan_backup(self):
        """fallback planning"""
        if self.idx < len(self.survivors):
            # simple fallback: visit next in sequence
            target = self.survivors[self.idx]
            
            if not self._exec(target.x, target.y):
                # if that fails, try closest
                self._exec_closest()

    def _exec_closest(self):
        """rescue closest unvisited survivor"""
        if not self.pose or self.idx >= len(self.survivors):
            return
            
        remaining_survivors = self.survivors[self.idx:]
        if not remaining_survivors:
            return
            
        # find closest
        min_dist = float('inf')
        closest_survivor = remaining_survivors[0]
        
        for survivor in remaining_survivors:
            dist = robot_utils.euclidean_distance(
                self.pose.position.x, self.pose.position.y,
                survivor.x, survivor.y
            )
            if dist < min_dist:
                min_dist = dist
                closest_survivor = survivor
        
        self._exec(closest_survivor.x, closest_survivor.y)

    def _exec_simple(self):
        """simple sequential rescue"""
        if self.idx < len(self.survivors):
            target = self.survivors[self.idx]
            self._exec(target.x, target.y)

    def _exec(self, target_x, target_y):
        """execute rescue to coords"""
        try:
            # create path request
            freq = FollowPathRequest()
            freq.path_x = [target_x]
            freq.path_y = [target_y]
            
            # send path
            fresp = self.path_srv(freq)
            if not fresp.success:
                self.planning_failures += 1
                return False
                
            self.state = RescueState.EXECUTING
            self._pub_status()
            self._pub_viz(target_x, target_y)
            return True
            
        except Exception as e:
            rospy.logerr(f"rescue execution failed: {e}")
            self.planning_failures += 1
            return False
    
    def _pub_status(self):
        """publish status"""
        msg = String()
        
        if self.state == RescueState.WAITING:
            msg.data = "waiting for exploration..."
        elif self.state == RescueState.PLANNING:
            msg.data = f"planning rescue {self.idx+1}/{len(self.survivors)}"
            if self.planning_failures > 0:
                msg.data += f" (failures: {self.planning_failures})"
        elif self.state == RescueState.EXECUTING:
            msg.data = f"executing rescue {self.idx+1}/{len(self.survivors)}"
        elif self.state == RescueState.DONE:
            msg.data = "rescue complete!"
            
        self.status_pub.publish(msg)
    
    def _pub_viz(self, x, y):
        """publish viz"""
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = "rescue_target"
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.0
        m.scale.x = 0.4
        m.scale.y = 0.4
        m.scale.z = 0.1
        m.color.r = 1.0
        m.color.g = 0.5
        m.color.b = 0.0  # orange for active target
        m.color.a = 0.8
        self.viz_pub.publish(m)

if __name__ == '__main__':
    try:
        robot = Rescue()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass