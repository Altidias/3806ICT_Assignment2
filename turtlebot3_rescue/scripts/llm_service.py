#!/usr/bin/env python3
import rospy
import requests
from turtlebot3_rescue.srv import OptimizePath, OptimizePathResponse
from geometry_msgs.msg import PoseArray

# xAI API configuration
XAI_API_KEY = "xai-6bcBZfRRT7LN8C0zqorobSrZ7faLzsdwGtfXr06EewQAWOIXwseEybQByWiSLocjHktoAJicCsuOW3dK"  # Don't steal my API key fellas, pay for your own subscription

XAI_API_URL = "https://api.x.ai/v1/chat/completions"  # Adjust based on actual API endpoint
headers = {
        "Authorization": f"Bearer {XAI_API_KEY}",
        "Content-Type": "application/json"

    }

def query_grok(prompt):
    payload = {
        "model": "grok-3-latest",
        "messages": [
            {"role": "system", "content": "You are Grok, an AI assistant for a rescue robot."},
            {"role": "user", "content": prompt}
        ],
        "max_tokens": 50,
        "temperature": 0.5
    }
    rospy.loginfo("Requesting URL: %s with payload: %s", XAI_API_URL, str(payload))
    try:
        response = requests.post(XAI_API_URL, headers=headers, json=payload)
        response.raise_for_status()
        json_response = response.json()
        rospy.loginfo("Raw API response: %s", str(json_response))
        if "choices" in json_response and json_response["choices"]:
            return json_response["choices"][0]["message"]["content"]
        else:
            rospy.logerr("Unexpected response format: no 'choices' or empty")
            return None
    except requests.exceptions.RequestException as e:
        rospy.logerr(f"Failed to query xAI API: {str(e)}")
        return None
    except ValueError as e:
        rospy.logerr(f"Invalid JSON response: {str(e)}")
        return None

def handle_optimize_path(req):
    rospy.loginfo("Received goal selection request with %d targets", len(req.targets.poses))
    resp = OptimizePathResponse()

    if not req.targets.poses:
        return resp

    # Prepare a prompt for Grok
    targets = req.targets.poses
    prompt = "You are Grok, an AI assistant for a rescue robot. Given the following coordinates of people to rescue, select the next person to visit based on proximity to the origin (0,0) and priority. Format: (x, y, priority). "
    for i, pose in enumerate(targets):
        priority = 1.0 / (i + 1)  # Mock priority; adjust as needed
        prompt += f"Person {i}: ({pose.position.x}, {pose.position.y}, {priority}); "
    prompt += "Return the person number (e.g., 0, 1, 2) to visit next in a single integer format."

    # Call xAI's Grok API
    grok_response = query_grok(prompt)
    if grok_response is None:
        rospy.logerr("Failed to get response from Grok API, falling back to default selection")
        closest_idx = 0
    else:
        try:
            closest_idx = int(grok_response.strip())
            if closest_idx < 0 or closest_idx >= len(targets):
                rospy.logwarn("Grok returned invalid index %d, defaulting to 0", closest_idx)
                closest_idx = 0
        except ValueError:
            rospy.logwarn("Grok response '%s' is not a valid number, defaulting to 0", grok_response)
            closest_idx = 0

    # Return the selected target
    selected_target = PoseArray()
    selected_target.header.frame_id = "map"
    selected_target.poses.append(targets[closest_idx])
    resp.optimized_targets = selected_target
    rospy.loginfo("Selected Person %d at (x=%f, y=%f)", closest_idx, 
                  targets[closest_idx].position.x, targets[closest_idx].position.y)
    return resp

def llm_server():
    rospy.init_node('llm_service')
    s = rospy.Service('/llm_optimize_path', OptimizePath, handle_optimize_path)
    rospy.loginfo("LLM service ready for goal selection with Grok")
    rospy.spin()

if __name__ == "__main__":
    llm_server()
