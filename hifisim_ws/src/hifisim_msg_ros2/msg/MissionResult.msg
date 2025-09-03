builtin_interfaces/Time stamp
string result_type   # Type of result: 'completed' and 'ooi_found'
# Below applicable only for result_type 'completed' 
# a valid agent_id means the agent has completed its mission
# agent_id = 0 mean all the agents have completed their mission 
int32 agent_id
# Below applicable only for result_type 'ooi_found' 
int32 ooi_id
int32 ooi_class_id
geometry_msgs/Point ooi_location