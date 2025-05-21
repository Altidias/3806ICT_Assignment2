#!/usr/bin/env python3

import rospy
import requests
import json
import time
from geometry_msgs.msg import PoseArray, PoseStamped
from turtlebot3_explorer.msg import AgentState, AbstractMap, ExplorationPlan

class LLMPlanner:
    """Ollama for high-level planning decisions"""
    
    def __init__(self):
        rospy.init_node('llm_planner')
        
        self.ollama_url = rospy.get_param('~ollama_url', 'http://localhost')
        
    
    def agent_state_callback(self, msg):
        
    
    def abstract_map_callback(self, msg):
        
    
    def frontiers_callback(self, msg):
        
    
    def planning_cycle(self, event):
        
    
    def create_prompt(self):
        prompt = f"""
You are an exploration planner for a robot. Your goal is to help the robot explore unknown areas efficiently.

...

Provide response as:
SELECTED_FRONTIER: <number of the frontier>
REASONING: <brief explanation of your choice>
"""
        return prompt
    
    def query_ollama(self, prompt):
        
    
    def process_llm_response(self, response):
        

if __name__ == '__main__':
    try:
        planner = LLMPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass