#!/usr/bin/env python3
import rospy
import requests
from turtlebot3_explorer.srv import OptimizePath, OptimizePathResponse
from geometry_msgs.msg import PoseArray

# xai api config
XAI_API_KEY = "xai-6bcBZfRRT7LN8C0zqorobSrZ7faLzsdwGtfXr06EewQAWOIXwseEybQByWiSLocjHktoAJicCsuOW3dK"

# global flag to track fallback mode
USE_FALLBACK_MODE = False

XAI_API_URL = "https://api.x.ai/v1/chat/completions"
headers = {
    "Authorization": f"Bearer {XAI_API_KEY}",
    "Content-Type": "application/json"
}

def query(prompt):
    payload = {
        "model": "grok-1",
        "messages": [
            {
                "role": "system",
                "content": "You are an AI assistant for a rescue robot. When given coordinates of rescue targets, respond ONLY with comma-separated numbers representing the optimal order to visit them (e.g., '2,0,1'). Consider distance, clustering, and efficient movement patterns."
            },
            {"role": "user", "content": prompt}
        ],
        "max_tokens": 20,  # just enough for indices
        "temperature": 0.1  # low for consistent results
    }
    
    try:
        response = requests.post(XAI_API_URL, headers=headers, json=payload, timeout=5.0)
        response.raise_for_status()
        json_response = response.json()
        
        if "choices" in json_response and json_response["choices"]:
            content = json_response["choices"][0]["message"]["content"].strip()
            return content
        else:
            rospy.logerr(f"unexpected api response format: {json_response}")
            return None
            
    except requests.exceptions.Timeout:
        rospy.logerr("api request timed out")
        return None
    except requests.exceptions.RequestException as e:
        rospy.logerr(f"api request failed: {str(e)}")
        return None
    except (ValueError, KeyError) as e:
        rospy.logerr(f"failed to parse api response: {str(e)}")
        return None

def optimize(req):
    global USE_FALLBACK_MODE
    
    resp = OptimizePathResponse()
    resp.optimized_targets = PoseArray()
    
    if not req.targets.poses:
        return resp

    if USE_FALLBACK_MODE:
        # simple sequential ordering
        resp.optimized_targets = req.targets
        return resp

    # build optimization prompt
    prompt = (
        "Task: optimize the visit order for the following target locations to minimize total travel distance "
        "and maximize rescue efficiency.\n\n"
        "Target locations (index: x, y):\n"
    )
    
    for i, pose in enumerate(req.targets.poses):
        prompt += f"{i}: ({pose.position.x:.2f}, {pose.position.y:.2f})\n"
    
    prompt += (
        "\nConsiderations:\n"
        "1. Minimize total path length\n"
        "2. Consider target clustering\n"
        "3. Account for efficient movement patterns\n"
        "\nrespond only with optimal visit order as comma-separated indices."
    )

    # get optimization with retries
    max_retries = 2
    for attempt in range(max_retries):
        grok_response = query(prompt)
        if grok_response is None:
            if attempt == max_retries - 1:
                # switch to permanent fallback mode
                USE_FALLBACK_MODE = True
                resp.optimized_targets = req.targets  # use original order as fallback
                return resp
            continue
        
        try:
            # parse and validate indices
            indices = [int(idx.strip()) for idx in grok_response.split(',') if idx.strip()]
            valid_indices = []
            
            # validate each index
            for idx in indices:
                if 0 <= idx < len(req.targets.poses):
                    if idx not in valid_indices:  # avoid duplicates
                        valid_indices.append(idx)
            
            # add any missing targets at the end
            all_indices = set(range(len(req.targets.poses)))
            missing_indices = all_indices - set(valid_indices)
            valid_indices.extend(sorted(missing_indices))
            
            # create the optimized path
            for idx in valid_indices:
                resp.optimized_targets.poses.append(req.targets.poses[idx])
            
            return resp
            
        except ValueError as e:
            if attempt == max_retries - 1:
                # switch to permanent fallback mode
                USE_FALLBACK_MODE = True
                resp.optimized_targets = req.targets  # use original order as fallback
                
    return resp

def server():
    rospy.init_node('llm_rescue_planner')
    s = rospy.Service('/llm_optimize_path', OptimizePath, optimize)
    rospy.loginfo("llm rescue planner ready")
    rospy.spin()

if __name__ == "__main__":
    server()