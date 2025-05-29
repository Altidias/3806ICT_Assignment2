#!/usr/bin/env python3

import rospy
import requests
import json
import math
from turtlebot3_explorer.srv import PlanRescuePath, PlanRescuePathRequest, PlanRescuePathResponse
from turtlebot3_explorer.srv import PlanPath, PlanPathRequest
from std_msgs.msg import String

# ollama config
OLLAMA_URL = "http://192.168.0.222:11434/api/generate"
OLLAMA_MODEL = "llama3.2:3b"

def query_llm(prompt, model=OLLAMA_MODEL):
    """query ollama"""
    payload = {
        "model": model,
        "prompt": prompt,
        "stream": False,
        "options": {
            "temperature": 0.2,
            "top_p": 0.9,
            "num_predict": 500
        }
    }
    
    try:
        response = requests.post(OLLAMA_URL, json=payload, timeout=30)
        response.raise_for_status()
        data = response.json()
        
        if "response" in data:
            return data["response"]
        else:
            return None
            
    except Exception as e:
        rospy.logerr(f"llm query failed: {str(e)}")
        return None

def dist(x1, y1, x2, y2):
    """calc distance"""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def make_prompt(start_x, start_y, survivors):
    """build rescue prompt"""
    prompt = """You are planning a rescue route.
Visit all survivors and return to start.

ROBOT START: ({:.2f}, {:.2f})

SURVIVORS:
""".format(start_x, start_y)
    
    # add survivors
    for i, (sx, sy) in enumerate(survivors):
        d = dist(start_x, start_y, sx, sy)
        prompt += f"- S{i}: ({sx:.2f}, {sy:.2f}) - {d:.2f}m from base\n"
    
    # distance matrix
    prompt += "\nDISTANCES:\n"
    n = len(survivors)
    
    for i in range(n):
        prompt += f"S{i}: "
        dists = []
        for j in range(n):
            if i != j:
                d = dist(survivors[i][0], survivors[i][1], survivors[j][0], survivors[j][1])
                dists.append(f"S{j}:{d:.1f}m")
        prompt += ", ".join(dists[:3]) + "\n"
    
    prompt += """
Return ONLY JSON:
{
  "visit_order": [indices],
  "reasoning": "why"
}
"""
    
    return prompt

def parse_response(resp_text, n_survivors):
    """extract visit order"""
    try:
        # find json
        start = resp_text.find('{')
        end = resp_text.rfind('}') + 1
        
        if start >= 0 and end > start:
            json_str = resp_text[start:end]
            data = json.loads(json_str)
            
            order = data.get("visit_order", [])
            
            # validate
            if len(order) == n_survivors and all(0 <= idx < n_survivors for idx in order):
                return order
            else:
                return None
        else:
            return None
            
    except Exception as e:
        rospy.logerr(f"parse error: {e}")
        return None

class LLMRescuePlanner:
    def __init__(self):
        rospy.init_node('llm_rescue_planner')
        
        # path planner client
        rospy.wait_for_service('/astar_planner/plan_path')
        self.path_client = rospy.ServiceProxy('/astar_planner/plan_path', PlanPath)
        
        # service - use same name as original rescue planner
        self.service = rospy.Service('/rescue_planner/plan_rescue_path', PlanRescuePath, self.handle_rescue)
        
        rospy.loginfo("llm rescue planner ready")
        
    def handle_rescue(self, req):
        """plan rescue path with llm"""
        resp = PlanRescuePathResponse()
        
        # get survivors
        survivors = list(zip(req.survivor_x, req.survivor_y))
        
        if not survivors:
            resp.success = False
            return resp
        
        # get llm visit order
        prompt = make_prompt(req.start_x, req.start_y, survivors)
        llm_resp = query_llm(prompt)
        
        if llm_resp:
            order = parse_response(llm_resp, len(survivors))
        else:
            order = None
        
        # fallback to greedy if llm fails
        if order is None:
            rospy.logwarn("llm failed, using greedy order")
            order = list(range(len(survivors)))
        
        # build actual path using a*
        full_path_x = [req.start_x]
        full_path_y = [req.start_y]
        
        # current position starts at robot start
        curr_x, curr_y = req.start_x, req.start_y
        
        # visit each survivor in order
        for idx in order:
            next_x, next_y = survivors[idx]
            
            # get path segment
            segment = self.get_path_segment(curr_x, curr_y, next_x, next_y, req)
            
            if segment:
                # add segment (skip first to avoid duplicates)
                for i in range(1, len(segment[0])):
                    full_path_x.append(segment[0][i])
                    full_path_y.append(segment[1][i])
                
                curr_x, curr_y = next_x, next_y
            else:
                resp.success = False
                return resp
        
        # return to start
        segment = self.get_path_segment(curr_x, curr_y, req.start_x, req.start_y, req)
        if segment:
            for i in range(1, len(segment[0])):
                full_path_x.append(segment[0][i])
                full_path_y.append(segment[1][i])
        
        # prepare response
        resp.path_x = full_path_x
        resp.path_y = full_path_y
        resp.visit_order = order
        resp.success = True
        
        return resp
    
    def get_path_segment(self, from_x, from_y, to_x, to_y, req):
        """get path segment using a*"""
        # convert to map coords
        start_mx = int((from_x - req.map_origin_x) / req.map_resolution)
        start_my = int((from_y - req.map_origin_y) / req.map_resolution)
        goal_mx = int((to_x - req.map_origin_x) / req.map_resolution)
        goal_my = int((to_y - req.map_origin_y) / req.map_resolution)
        
        # call a* planner
        path_req = PlanPathRequest()
        path_req.robot_id = "rescue_robot"
        path_req.start_x = start_mx
        path_req.start_y = start_my
        path_req.goal_x = goal_mx
        path_req.goal_y = goal_my
        path_req.map_data = req.map_data
        path_req.map_width = req.map_width
        path_req.map_height = req.map_height
        path_req.use_heuristics = False
        
        try:
            path_resp = self.path_client(path_req)
            
            if path_resp.success:
                # convert back to world coords
                world_x = []
                world_y = []
                for i in range(len(path_resp.path_x)):
                    wx = (path_resp.path_x[i] + 0.5) * req.map_resolution + req.map_origin_x
                    wy = (path_resp.path_y[i] + 0.5) * req.map_resolution + req.map_origin_y
                    world_x.append(wx)
                    world_y.append(wy)
                
                return (world_x, world_y)
        except Exception as e:
            rospy.logerr(f"path planning failed: {e}")
        
        return None

def main():
    try:
        planner = LLMRescuePlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()