#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from threading import Lock
import copy

class MapMerger:
    def __init__(self):
        rospy.init_node('map_merger')
        
        # params
        self.robots = rospy.get_param('~robot_names', ['robot1', 'robot2'])
        self.out_topic = rospy.get_param('~merged_map_topic', '/map')
        self.rate = rospy.get_param('~merge_rate', 30.0)
        self.size = rospy.get_param('~map_size', 30.0)  # meters
        self.res = rospy.get_param('~resolution', 0.1)  # m/pixel
        
        # state
        self.maps = {}
        self.lock = Lock()
        self.global_map = None
        
        # tf
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        
        # pub/sub
        self.pub = rospy.Publisher(self.out_topic, OccupancyGrid, queue_size=1, latch=True)
        
        self.subs = {}
        for robot in self.robots:
            topic = f'/{robot}/map'
            self.subs[robot] = rospy.Subscriber(
                topic, OccupancyGrid, 
                lambda msg, n=robot: self.map_cb(msg, n)
            )
            self.maps[robot] = None
        
        # init global map
        self.init_map()
        
        # merge timer
        self.timer = rospy.Timer(rospy.Duration(1.0/self.rate), self.merge_cb)

    def map_cb(self, msg, robot):
        with self.lock:
            self.maps[robot] = msg

    def init_map(self):
        """init global map"""
        self.global_map = OccupancyGrid()
        self.global_map.header.frame_id = "map"
        
        # map params
        cells = int(self.size / self.res)
        self.global_map.info.width = cells
        self.global_map.info.height = cells
        self.global_map.info.resolution = self.res
        
        # center origin
        self.global_map.info.origin.position.x = -self.size / 2.0
        self.global_map.info.origin.position.y = -self.size / 2.0
        self.global_map.info.origin.position.z = 0.0
        self.global_map.info.origin.orientation.w = 1.0
        
        # init unknown
        total = cells * cells
        self.global_map.data = [-1] * total

    def merge_cb(self, event):
        """merge all maps"""
        with self.lock:
            valid = []
            
            # collect valid maps
            for robot in self.robots:
                if self.maps[robot] is not None:
                    tf = self.get_tf(robot)
                    if tf is not None:
                        valid.append((robot, copy.deepcopy(self.maps[robot]), tf))
            
            if not valid:
                return
            
            # merge
            merged = self.merge(valid)
            
            # publish
            self.global_map.header.stamp = rospy.Time.now()
            self.global_map.data = merged.flatten().tolist()
            self.pub.publish(self.global_map)

    def get_tf(self, robot):
        """get transform"""
        try:
            # try actual transform
            tf = self.tf_buf.lookup_transform(
                'map', f'{robot}/map', 
                rospy.Time(0), rospy.Duration(1.0)
            )
            return tf.transform
        except:
            # fallback to spawn pos
            x = rospy.get_param(f'/{robot}_spawn_x', 0.0)
            y = rospy.get_param(f'/{robot}_spawn_y', 0.0)
            
            from geometry_msgs.msg import Transform
            tf = Transform()
            tf.translation.x = x
            tf.translation.y = y
            tf.translation.z = 0.0
            tf.rotation.w = 1.0
            
            return tf

    def merge(self, valid):
        """merge maps"""
        w = self.global_map.info.width
        h = self.global_map.info.height
        
        # voting arrays
        votes = np.zeros((h, w, 3), dtype=np.int32)  # [unknown, free, occupied]
        
        for robot, map, tf in valid:
            self.add_votes(map, tf, votes)
        
        # create final map
        merged = np.full((h, w), -1, dtype=np.int8)
        
        for y in range(h):
            for x in range(w):
                v = votes[y, x]
                total = np.sum(v)
                
                if total > 0:
                    if v[2] > v[1]:  # occupied
                        merged[y, x] = 100
                    elif v[1] > 0:  # free
                        merged[y, x] = 0
        
        return merged

    def add_votes(self, local_map, tf, votes):
        """add local map to votes"""
        # offset
        off_x = tf.translation.x
        off_y = tf.translation.y
        
        # process cells
        data = np.array(local_map.data).reshape(
            local_map.info.height, local_map.info.width
        )
        
        for ly in range(local_map.info.height):
            for lx in range(local_map.info.width):
                val = data[ly, lx]
                
                # local to world
                local_x = (local_map.info.origin.position.x + 
                          (lx + 0.5) * local_map.info.resolution)
                local_y = (local_map.info.origin.position.y + 
                          (ly + 0.5) * local_map.info.resolution)
                
                # apply transform
                world_x = local_x + off_x
                world_y = local_y + off_y
                
                # to global map coords
                gx = int((world_x - self.global_map.info.origin.position.x) / 
                        self.global_map.info.resolution)
                gy = int((world_y - self.global_map.info.origin.position.y) / 
                        self.global_map.info.resolution)
                
                # bounds check
                if (0 <= gx < self.global_map.info.width and 
                    0 <= gy < self.global_map.info.height):
                    
                    # vote
                    if val == -1:  # unknown
                        votes[gy, gx, 0] += 1
                    elif val == 0:  # free
                        votes[gy, gx, 1] += 1
                    else:  # occupied
                        votes[gy, gx, 2] += 1

if __name__ == '__main__':
    try:
        merger = MapMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass