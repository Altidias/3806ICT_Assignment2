#!/usr/bin/env python3

import math
import numpy as np

def world_to_map(world_x, world_y, map_info):
    """convert world coords to map grid coords"""
    if not map_info:
        return None, None
    mx = int((world_x - map_info.origin.position.x) / map_info.resolution)
    my = int((world_y - map_info.origin.position.y) / map_info.resolution)
    return mx, my

def map_to_world(map_x, map_y, map_info):
    """convert map grid coords to world coords - center of cell not corner"""
    if not map_info:
        return 0.0, 0.0
    wx = (map_x + 0.5) * map_info.resolution + map_info.origin.position.x
    wy = (map_y + 0.5) * map_info.resolution + map_info.origin.position.y
    return wx, wy

def euclidean_distance(x1, y1, x2, y2):
    """calc euclidean distance"""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def normalize_angle(angle):
    """normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def get_yaw_from_quaternion(q):
    """extract yaw from quaternion"""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 
                     1 - 2 * (q.y * q.y + q.z * q.z))