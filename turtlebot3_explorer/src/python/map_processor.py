#!/usr/bin/env python3

import rospy
import numpy as np
import json
from turtlebot3_explorer.msg import SensorData, AgentState, AbstractMap
from geometry_msgs.msg import PoseStamped

class MapProcessor:
    """Process sensor data into environmentl state representation"""
    
    def __init__(self):
        rospy.init_node('map_processor')
        
        self.current_map = None
        self.explored_area = 0.0
        self.total_area = 0.0
        
        self.state_pub = rospy.Publisher('/agent_state', AgentState, queue_size=1)
        self.abstract_map_pub = rospy.Publisher('/abstract_map', AbstractMap, queue_size=1)
        
        rospy.Subscriber('/sensor_data', SensorData, self.process_data)
        

    
    def process_data(self, data):
        """Process incoming sensor data"""
        self.current_map = data.map_data
        
        self.process_map_statistics()
        
        abstract_map = self.create_abstract_map()
        
        self.publish_agent_state(data.robot_pose, abstract_map)
    
    def process_map_statistics(self):
        """Calculate map statistics like exploration percentage"""
        if self.current_map is None:
            return
            
        free_cells = 0
        occupied_cells = 0
        unknown_cells = 0
        
        for cell in self.current_map.data:
            if cell == 0:  # free
                free_cells += 1
            elif cell == 100:  # occupied
                occupied_cells += 1
            elif cell == -1:  # unknown
                unknown_cells += 1
        
        total_cells = free_cells + occupied_cells + unknown_cells
        self.explored_area = (free_cells + occupied_cells) / total_cells if total_cells > 0 else 0
        self.total_area = total_cells * (self.current_map.info.resolution ** 2)
        
    
    def create_abstract_map(self):
        """Create an abstract representation of the map for the LLM"""
        if self.current_map is None:
            return None
            
        # simplified grid representation
        width = self.current_map.info.width
        height = self.current_map.info.height
        resolution = self.current_map.info.resolution
        
        if width > 50 or height > 50:
            downsample_factor = max(1, min(width // 50, height // 50))
            abstract_width = width // downsample_factor
            abstract_height = height // downsample_factor
            
            abstract_grid = np.zeros((abstract_height, abstract_width), dtype=int)
            
            for y in range(abstract_height):
                for x in range(abstract_width):
                    orig_y = y * downsample_factor
                    orig_x = x * downsample_factor
                    
                    # dominant cell type in this area
                    area = []
                    for dy in range(downsample_factor):
                        for dx in range(downsample_factor):
                            if orig_y + dy < height and orig_x + dx < width:
                                idx = (orig_y + dy) * width + (orig_x + dx)
                                area.append(self.current_map.data[idx])
                    
                    if area.count(0) > len(area) // 2:
                        abstract_grid[y, x] = 0  # free
                    elif area.count(100) > len(area) // 4:
                        abstract_grid[y, x] = 1  # occupied
                    else:
                        abstract_grid[y, x] = 2  # unknown
        else:
            abstract_grid = np.zeros((height, width), dtype=int)
            for y in range(height):
                for x in range(width):
                    idx = y * width + x
                    if self.current_map.data[idx] == 0:
                        abstract_grid[y, x] = 0  # free
                    elif self.current_map.data[idx] == 100:
                        abstract_grid[y, x] = 1  # occupied
                    else:
                        abstract_grid[y, x] = 2  # unknown
        
        return {
            'grid': abstract_grid.tolist(),
            'resolution': resolution * (downsample_factor if 'downsample_factor' in locals() else 1),
            'origin_x': self.current_map.info.origin.position.x,
            'origin_y': self.current_map.info.origin.position.y
        }
        
    def publish_agent_state(self, robot_pose, abstract_map):
        if abstract_map is None:
            return
            
        state = AgentState()
        state.header.stamp = rospy.Time.now()
        state.robot_pose = robot_pose
        state.exploration_percentage = self.explored_area * 100
        
        map_msg = AbstractMap()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.data = json.dumps(abstract_map)
        
        self.state_pub.publish(state)
        self.abstract_map_pub.publish(map_msg)
        

if __name__ == '__main__':
    try:
        processor = MapProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass