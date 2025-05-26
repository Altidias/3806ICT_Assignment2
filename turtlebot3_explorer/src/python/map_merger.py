#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
from threading import Lock
import copy

class MapMerger:
    """node to merge maps from multiple robots into a single global map"""
    
    def __init__(self):
        rospy.init_node('map_merger')
        
        self.robot_namespaces = rospy.get_param('~robot_namespaces', ['robot1', 'robot2'])
        self.merged_map_topic = rospy.get_param('~merged_map_topic', '/map')
        self.world_frame = rospy.get_param('~world_frame', 'map')
        self.merge_rate = rospy.get_param('~merge_rate', 2.0)
        
        rospy.loginfo("Map Merger initializing...")
        rospy.loginfo("Robot namespaces: %s", self.robot_namespaces)
        rospy.loginfo("Publishing merged map to: %s", self.merged_map_topic)
        
        # map storage
        self.robot_maps = {}
        self.robot_transforms = {}
        self.map_locks = {}
        self.global_map = None
        self.map_initialized = False
        
        # tf stuff
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.merged_map_pub = rospy.Publisher(self.merged_map_topic, OccupancyGrid, queue_size=1, latch=True)
        
        # sub to individual robot maps
        self.map_subs = {}
        for robot_ns in self.robot_namespaces:
            self.robot_maps[robot_ns] = None
            self.map_locks[robot_ns] = Lock()
            topic = f'/{robot_ns}/map'
            self.map_subs[robot_ns] = rospy.Subscriber(
                topic, OccupancyGrid, 
                lambda msg, ns=robot_ns: self.map_callback(msg, ns)
            )
            rospy.loginfo(f"Subscribing to {topic}")
        
        self.merge_timer = rospy.Timer(rospy.Duration(1.0 / self.merge_rate), self.merge_maps)
        
        rospy.loginfo("Map Merger initialized successfully!")
        
    def map_callback(self, msg, robot_ns):
        with self.map_locks[robot_ns]:
            self.robot_maps[robot_ns] = msg
            rospy.loginfo_once(f"Rec first map from {robot_ns}")
            
    def get_robot_transform(self, robot_ns):
        try:
            source_frame = f"{robot_ns}/map"
            target_frame = self.world_frame
            
            if self.tf_buffer.can_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(0.1)):
                transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0))
                return transform
            else:
                rospy.logwarn_once(f"Transform from {source_frame} to {target_frame} not available, using fallback")
                
                transform = TransformStamped()
                transform.header.frame_id = target_frame
                transform.child_frame_id = source_frame
                transform.header.stamp = rospy.Time.now()
                
                # robot starting positions should match spawn positions defined
                if robot_ns == 'robot1':
                    transform.transform.translation.x = -2.0
                    transform.transform.translation.y = 0.0
                elif robot_ns == 'robot2':
                    transform.transform.translation.x = -2.0
                    transform.transform.translation.y = 1.0
                elif robot_ns == 'robot3':
                    transform.transform.translation.x = -2.0
                    transform.transform.translation.y = -1.0
                else:
                    transform.transform.translation.x = 0.0
                    transform.transform.translation.y = 0.0
                    
                transform.transform.translation.z = 0.0
                transform.transform.rotation.w = 1.0
                
                return transform
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"Failed to get transform for {robot_ns}: {e}")
            return None
            
    def world_to_map_index(self, world_x, world_y, map_info):
        mx = int((world_x - map_info.origin.position.x) / map_info.resolution)
        my = int((world_y - map_info.origin.position.y) / map_info.resolution)
        return mx, my
        
    def map_to_world(self, mx, my, map_info):
        wx = map_info.origin.position.x + (mx + 0.5) * map_info.resolution
        wy = map_info.origin.position.y + (my + 0.5) * map_info.resolution
        return wx, wy
        
    def initialize_global_map(self, reference_map):
        self.global_map = OccupancyGrid()
        self.global_map.header.frame_id = self.world_frame
        self.global_map.info = copy.deepcopy(reference_map.info)
        
        expansion_factor = 3
        self.global_map.info.width = reference_map.info.width * expansion_factor
        self.global_map.info.height = reference_map.info.height * expansion_factor
        
        # center map origin
        map_size_x = self.global_map.info.width * self.global_map.info.resolution
        map_size_y = self.global_map.info.height * self.global_map.info.resolution
        self.global_map.info.origin.position.x = -map_size_x / 2.0
        self.global_map.info.origin.position.y = -map_size_y / 2.0
        self.global_map.info.origin.position.z = 0.0
        
        self.global_map.data = [-1] * (self.global_map.info.width * self.global_map.info.height)
        self.map_initialized = True
        
        rospy.loginfo(f"Initialized global map: {self.global_map.info.width}x{self.global_map.info.height} "
                     f"at resolution {self.global_map.info.resolution}")
        
    def merge_maps(self, event):
        """merge all robot maps into a global map"""
        valid_maps = []
        transforms = []
        
        for robot_ns in self.robot_namespaces:
            with self.map_locks[robot_ns]:
                if self.robot_maps[robot_ns] is not None:
                    transform = self.get_robot_transform(robot_ns)
                    if transform:
                        valid_maps.append((robot_ns, copy.deepcopy(self.robot_maps[robot_ns])))
                        transforms.append(transform)
                    else:
                        # if no transform available yet, skip this robo for now
                        rospy.logdebug(f"No transform available for {robot_ns}, skipping")
                        
        if not valid_maps:
            if rospy.Time.now().to_sec() > 10.0:
                rospy.logwarn_throttle(5.0, "No valid maps with transforms available yet")
            return
            
        if not self.map_initialized:
            self.initialize_global_map(valid_maps[0][1])
            
        width = self.global_map.info.width
        height = self.global_map.info.height
        
        occupied_sum = np.zeros((height, width), dtype=np.float32)
        occupied_count = np.zeros((height, width), dtype=np.int32)
        
        # process each robots map
        for (robot_ns, local_map), transform in zip(valid_maps, transforms):
            local_data = np.array(local_map.data, dtype=np.int8).reshape(
                local_map.info.height, local_map.info.width
            )
            
            # transform offset
            offset_x = transform.transform.translation.x
            offset_y = transform.transform.translation.y
            
            
            # proces each cell in the local map
            for ly in range(local_map.info.height):
                for lx in range(local_map.info.width):
                    # get local cell value
                    value = local_data[ly, lx]
                    
                    # skip unknown cells
                    if value == -1:
                        continue
                        
                    # convert local map coordinates to world coordinates
                    local_wx, local_wy = self.map_to_world(lx, ly, local_map.info)
                    
                    # apply transform to get global world coordinates
                    global_wx = local_wx + offset_x
                    global_wy = local_wy + offset_y
                    
                    # convert to global map indices
                    gx, gy = self.world_to_map_index(global_wx, global_wy, self.global_map.info)
                    
                    # check bounds
                    if 0 <= gx < width and 0 <= gy < height:
                        # accumulate values for averaging
                        occupied_sum[gy, gx] += value
                        occupied_count[gy, gx] += 1
                        
        # final merged map
        merged_data = np.full((height, width), -1, dtype=np.int8)
        
        # calculate averaged values
        mask = occupied_count > 0
        merged_data[mask] = np.round(occupied_sum[mask] / occupied_count[mask]).astype(np.int8)
        
        # thresholds for cleaner map
        merged_data[merged_data > 65] = 100  # Occupied
        merged_data[(merged_data >= 0) & (merged_data < 20)] = 0  # Free
        # vcalues between 20 and 65 remain as is (uncertain)
        
        # update and publish global map
        self.global_map.header.stamp = rospy.Time.now()
        self.global_map.data = merged_data.flatten().tolist()
        
        self.merged_map_pub.publish(self.global_map)
        
        # statistics
        known_cells = np.sum(mask)
        total_cells = width * height
        occupied_cells = np.sum(merged_data == 100)
        free_cells = np.sum(merged_data == 0)
        
        rospy.loginfo_throttle(5.0, 
            f"Merged {len(valid_maps)} maps | "
            f"Coverage: {100*known_cells/total_cells:.1f}% | "
            f"Free: {free_cells} | Occupied: {occupied_cells}"
        )

def main():
    try:
        merger = MapMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Map merger fail: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()