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
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from turtlebot3_explorer.srv import GetSurvivors, GetSurvivorsRequest, PlanRescuePath, PlanRescuePathRequest, FollowPath, FollowPathRequest

class RescueState(Enum):
    WAITING = "waiting"
    PLANNING = "planning"
    EXECUTING = "executing"
    DONE = "done"

class RescueRobot:
    def __init__(self):
        rospy.init_node('rescue_robot')
        
        # params
        self.name = rospy.get_param('~robot_name', 'rescue_robot')
        self.x0 = rospy.get_param('~spawn_x', 0.0)
        self.y0 = rospy.get_param('~spawn_y', 0.0)
        
        # state
        self.state = RescueState.WAITING
        self.pose = None
        self.map = None
        self.survivors = []
        self.idx = 0
        self.lock = Lock()
        
        # wait for services
        rospy.wait_for_service('/survivor_manager/get_survivors')
        self.survivor_srv = rospy.ServiceProxy('/survivor_manager/get_survivors', GetSurvivors)
        
        rospy.wait_for_service('/rescue_planner/plan_rescue_path')
        self.planner_srv = rospy.ServiceProxy('/rescue_planner/plan_rescue_path', PlanRescuePath)
        
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
        with self.lock:
            if msg.data and self.state == RescueState.WAITING:
                self.state = RescueState.PLANNING

    def _map_cb(self, msg):
        with self.lock:
            self.map = msg

    def _odom_cb(self, msg):
        with self.lock:
            self.pose = msg.pose.pose
            self._pub_marker()

    def _path_cb(self, msg):
        with self.lock:
            if msg.data and self.state == RescueState.EXECUTING:
                # next survivor
                self.idx += 1
                
                if self.idx >= len(self.survivors):
                    # check if back home
                    if self.pose:
                        d = robot_utils.euclidean_distance(
                            self.x0, self.y0,
                            self.pose.position.x, self.pose.position.y
                        )
                        if d < 0.5:
                            self.state = RescueState.DONE
                            
                            # done
                            msg = Bool()
                            msg.data = True
                            self.done_pub.publish(msg)

    def _update(self, event):
        with self.lock:
            if self.state == RescueState.PLANNING:
                self._plan()
            
            self._pub_status()

    def _plan(self):
        """plan route"""
        if not self.map or not self.pose:
            return
        
        # get survivors
        try:
            req = GetSurvivorsRequest()
            req.only_discovered = True
            resp = self.survivor_srv(req)
            
            if not resp.x_positions:
                self.state = RescueState.DONE
                return
            
            # plan path
            plan_req = PlanRescuePathRequest()
            plan_req.start_x = self.pose.position.x
            plan_req.start_y = self.pose.position.y
            plan_req.survivor_x = resp.x_positions
            plan_req.survivor_y = resp.y_positions
            plan_req.map_data = list(self.map.data)
            plan_req.map_width = self.map.info.width
            plan_req.map_height = self.map.info.height
            plan_req.map_resolution = self.map.info.resolution
            plan_req.map_origin_x = self.map.info.origin.position.x
            plan_req.map_origin_y = self.map.info.origin.position.y
            
            plan_resp = self.planner_srv(plan_req)
            
            if plan_resp.success:
                # store survivors
                self.survivors = list(zip(resp.x_positions, resp.y_positions))
                self.idx = 0
                
                # follow path
                follow_req = FollowPathRequest()
                follow_req.path_x = plan_resp.path_x
                follow_req.path_y = plan_resp.path_y
                
                follow_resp = self.path_srv(follow_req)
                
                if follow_resp.success:
                    self.state = RescueState.EXECUTING
                
        except Exception as e:
            rospy.logerr(f"plan error: {e}")

    def _pub_status(self):
        """pub status"""
        msg = String()
        
        if self.state == RescueState.WAITING:
            msg.data = "waiting for exploration"
        elif self.state == RescueState.PLANNING:
            msg.data = "planning route"
        elif self.state == RescueState.EXECUTING:
            msg.data = f"rescuing {self.idx}/{len(self.survivors)}"
        elif self.state == RescueState.DONE:
            msg.data = "rescue complete"
        
        self.status_pub.publish(msg)

    def _pub_marker(self):
        """pub viz"""
        if not self.pose:
            return
        
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = "rescue_robot"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        
        m.pose = self.pose
        m.pose.position.z = 0.1
        
        m.scale.x = 0.3
        m.scale.y = 0.3
        m.scale.z = 0.3
        
        # blue
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 1.0
        m.color.a = 1.0
        
        self.viz_pub.publish(m)

if __name__ == '__main__':
    try:
        rescue = RescueRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass