Header header
string robot_id
string status  # "idle", "moving", "reached_goal", "obstacle", "failed"
geometry_msgs/Pose current_pose
geometry_msgs/Pose target_pose
float32 progress_percentage