#!/usr/bin/env python3
import rospy
import requests
from turtlebot3_explorer.srv import SelectFrontiers, SelectFrontiersResponse
from geometry_msgs.msg import PoseArray

# xai api config
XAI_API_KEY = "xai-6bcBZfRRT7LN8C0zqorobSrZ7faLzsdwGtfXr06EewQAWOIXwseEybQByWiSLocjHktoAJicCsuOW3dK"

# global flag to track fallback mode
USE_FALLBACK_MODE = False

XAI_API_URL = "https://api.x.ai/v1/chat/completions"
XAI_API_URL_BACKUP = "https://api-prod.x.ai/v1/chat/completions"  # backup url
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
                "content": (
                    "You are an AI coordinator for multi-robot exploration. "
                    "Analyze robot positions and frontier points to maximize exploration efficiency. "
                    "Respond ONLY with a single number for the chosen frontier index. "
                    "Consider: 1) Distance to frontier 2) Robot distribution 3) Exploration potential"
                )
            },
            {"role": "user", "content": prompt}
        ],
        "max_tokens": 5,  # just enough for a single number
        "temperature": 0.1  # low for deterministic responses
    }
    
    try:
        # try primary url first
        try:
            response = requests.post(XAI_API_URL, headers=headers, json=payload, timeout=3.0)
            response.raise_for_status()
        except (requests.exceptions.RequestException, requests.exceptions.Timeout) as e:
            rospy.logwarn(f"primary api failed, trying backup: {str(e)}")
            response = requests.post(XAI_API_URL_BACKUP, headers=headers, json=payload, timeout=3.0)
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

def select_frontiers(req):
    global USE_FALLBACK_MODE
    
    resp = SelectFrontiersResponse()
    resp.success = True
    resp.frontier_indices = [-1] * len(req.robot_ids)
    resp.reasoning = [""] * len(req.robot_ids)
    
    if not req.frontiers.poses:
        return resp

    # fallback logic for when api is unreachable
    def get_closest(robot_pose, available_frontiers):
        """simple fallback to pick closest frontier if api fails"""
        min_dist = float('inf')
        closest_idx = 0
        for i, (_, frontier) in enumerate(available_frontiers):
            dx = robot_pose.position.x - frontier.position.x
            dy = robot_pose.position.y - frontier.position.y
            dist = dx * dx + dy * dy
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        return closest_idx

    # filter out assigned frontiers
    assigned = set(req.assigned_indices)
    available_frontiers = [(i, f) for i, f in enumerate(req.frontiers.poses) 
                          if i not in assigned]
    
    if not available_frontiers:
        return resp

    if USE_FALLBACK_MODE:
        # for each robot needing assignment
        for robot_idx, (robot_id, robot_pose) in enumerate(zip(req.robot_ids, req.robot_poses.poses)):
            closest_idx = get_closest(robot_pose, available_frontiers)
            frontier_idx = available_frontiers[closest_idx][0]
            resp.frontier_indices[robot_idx] = frontier_idx
            resp.reasoning[robot_idx] = f"selected closest frontier {frontier_idx} (permanent fallback mode)"
            available_frontiers.pop(closest_idx)
        return resp

    # for each robot needing assignment
    for robot_idx, (robot_id, robot_pose) in enumerate(zip(req.robot_ids, req.robot_poses.poses)):
        # build the task prompt
        prompt = (
            f"task: select the best frontier for robot {robot_id} at position "
            f"({robot_pose.position.x:.2f}, {robot_pose.position.y:.2f}).\n\n"
            "other robot positions:\n"
        )
        
        # add other robot positions for coordination
        for other_id, other_pose in zip(req.robot_ids, req.robot_poses.poses):
            if other_id != robot_id:
                prompt += f"- {other_id}: ({other_pose.position.x:.2f}, {other_pose.position.y:.2f})\n"
        
        # add available frontiers
        prompt += "\navailable frontiers (index, x, y):\n"
        for i, (frontier_idx, frontier) in enumerate(available_frontiers):
            prompt += f"{i}: ({frontier.position.x:.2f}, {frontier.position.y:.2f})\n"
        
        prompt += (
            "\nconsiderations:\n"
            "1. choose closest reachable frontier\n"
            "2. avoid selecting frontiers near other robots\n"
            "3. prefer frontiers that lead to unexplored areas\n"
            "\nrespond with only the frontier index number (0-N)."
        )

        # get frontier selection from grok
        retries = 2
        success = False
        while retries > 0:
            grok_response = query(prompt)
            if grok_response is None:
                retries -= 1
                continue
                
            try:
                list_idx = int(grok_response)
                if 0 <= list_idx < len(available_frontiers):
                    frontier_idx = available_frontiers[list_idx][0]
                    resp.frontier_indices[robot_idx] = frontier_idx
                    resp.reasoning[robot_idx] = f"selected frontier {frontier_idx} for optimal exploration"
                    available_frontiers.pop(list_idx)
                    success = True
                    break
                else:
                    retries -= 1
            except ValueError:
                retries -= 1
        
        if not success:
            # switch to permanent fallback mode
            USE_FALLBACK_MODE = True
            # fallback to closest frontier if api fails
            closest_idx = get_closest(robot_pose, available_frontiers)
            frontier_idx = available_frontiers[closest_idx][0]
            resp.frontier_indices[robot_idx] = frontier_idx
            resp.reasoning[robot_idx] = f"selected closest frontier {frontier_idx} (switching to permanent fallback)"
            available_frontiers.pop(closest_idx)

    return resp

def server():
    rospy.init_node('llm_frontier_selector')
    s = rospy.Service('/llm_select_frontiers', SelectFrontiers, select_frontiers)
    rospy.loginfo("llm frontier selector ready")
    rospy.spin()

if __name__ == "__main__":
    server()