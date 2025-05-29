#!/usr/bin/env python3

import rospy
import requests
import json
import math
from turtlebot3_explorer.srv import SelectFrontiers, SelectFrontiersResponse
from geometry_msgs.msg import PoseArray

# ollama config
OLLAMA_HOST = rospy.get_param('/ollama_host', '192.168.0.222')
OLLAMA_URL = f"http://{OLLAMA_HOST}:11434/api/generate"
OLLAMA_MODEL = "codellama:7b-instruct-q4_0"

def dist(x1, y1, x2, y2):
    """calc distance"""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def query_llm(prompt, model=OLLAMA_MODEL):
    """query ollama"""
    payload = {
        "model": model,
        "prompt": prompt,
        "stream": False,
        "options": {
            "temperature": 0.1,
            "top_p": 0.9,
            "num_predict": 200,
            "num_ctx": 1024,
            "stop": ["\n\n", "```", "Note:", "Remember:"]
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

def make_prompt(rids, rposes, frontiers, assigned):
    """build minimal prompt"""
    # robot info
    robots = []
    for i, (rid, pose) in enumerate(zip(rids, rposes.poses)):
        robots.append(f"{rid}:({pose.position.x:.1f},{pose.position.y:.1f})")
    
    # unassigned frontiers only
    available = []
    dists = {}
    
    for i, f in enumerate(frontiers.poses):
        if i not in assigned:
            available.append(f"{i}:({f.position.x:.1f},{f.position.y:.1f})")
            
            # calc distances
            for j, (rid, rp) in enumerate(zip(rids, rposes.poses)):
                d = dist(rp.position.x, rp.position.y, f.position.x, f.position.y)
                if rid not in dists:
                    dists[rid] = []
                dists[rid].append(f"{i}:{d:.1f}m")
    
    prompt = f"""You are in charge of an explorer robot using slam to map out an environment using frontiers. Assign frontiers to maximize exploration coverage and seperation between explorers.

Robots: {', '.join(robots)}
Frontiers: {', '.join(available[:10])}{'...' if len(available) > 10 else ''}

Distances:
"""
    
    for rid in rids:
        if rid in dists:
            prompt += f"{rid}: {', '.join(dists[rid][:5])}\n"
    
    prompt += """
OUTPUT ONLY THIS JSON (no other text):
{"assignments": {"""
    
    for i, rid in enumerate(rids):
        if i > 0:
            prompt += ", "
        prompt += f'"{rid}": <index>'
    
    prompt += """}, "reasoning": {"""
    
    for i, rid in enumerate(rids):
        if i > 0:
            prompt += ", "
        prompt += f'"{rid}": "<5 words>"'
    
    prompt += """}}"""
    
    return prompt

def parse_response(resp_text, rids):
    """extract assignments from llm response"""
    try:
        # clean up response
        cleaned = resp_text.strip()
        
        # remove markdown if present
        if "```json" in cleaned:
            start = cleaned.find("```json") + 7
            end = cleaned.find("```", start)
            if end > start:
                cleaned = cleaned[start:end].strip()
        elif "```" in cleaned:
            start = cleaned.find("```") + 3
            end = cleaned.find("```", start)
            if end > start:
                cleaned = cleaned[start:end].strip()
        
        # find json
        json_start = cleaned.find('{')
        json_end = cleaned.rfind('}') + 1
        
        if json_start >= 0 and json_end > json_start:
            json_str = cleaned[json_start:json_end]
            data = json.loads(json_str)
            
            assigns = []
            reasons = []
            
            assign_data = data.get("assignments", {})
            reason_data = data.get("reasoning", {})
            
            for rid in rids:
                if str(rid) in assign_data:
                    try:
                        assigns.append(int(assign_data[str(rid)]))
                    except:
                        assigns.append(-1)
                else:
                    assigns.append(-1)
                
                if str(rid) in reason_data:
                    reasons.append(str(reason_data[str(rid)])[:50])
                else:
                    reasons.append("no reason")
            
            return assigns, reasons
            
        else:
            return None, None
            
    except Exception as e:
        rospy.logerr(f"parse error: {e}")
        return None, None

def handle_select(req):
    """handle frontier selection request"""
    resp = SelectFrontiersResponse()
    
    if not req.robot_ids or not req.frontiers.poses:
        resp.success = False
        return resp
    
    # build prompt
    prompt = make_prompt(req.robot_ids, req.robot_poses, req.frontiers, req.assigned_indices)
    
    # query llm
    llm_resp = query_llm(prompt)
    
    if llm_resp:
        # parse
        assigns, reasons = parse_response(llm_resp, req.robot_ids)
        
        if assigns is not None:
            resp.frontier_indices = assigns
            resp.reasoning = reasons
            resp.success = True
            return resp
    
    # llm failed - return empty
    resp.success = False
    resp.frontier_indices = [-1] * len(req.robot_ids)
    resp.reasoning = ["llm failed"] * len(req.robot_ids)
    return resp

def llm_service():
    """run llm frontier service"""
    rospy.init_node('llm_frontier_selector')
    
    global OLLAMA_HOST
    OLLAMA_HOST = rospy.get_param('~ollama_host', OLLAMA_HOST)
    
    rospy.loginfo(f"llm frontier selector starting on {OLLAMA_HOST}")
    
    # create service
    service = rospy.Service('/llm_select_frontiers', SelectFrontiers, handle_select)
    
    rospy.loginfo("ready")
    rospy.spin()

if __name__ == "__main__":
    try:
        llm_service()
    except rospy.ROSInterruptException:
        pass