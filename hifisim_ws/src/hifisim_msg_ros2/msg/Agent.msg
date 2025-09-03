# Define an individual agent
# Use Unity C# naming convention CamelCase because data read from Unity created json
int32 agent_id
string ip_address
geometry_msgs/Pose initial_pose 
Waypoint[] waypoints
