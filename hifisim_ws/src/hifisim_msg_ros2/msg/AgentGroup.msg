# Define an individual agent group
# Use Unity C# naming convention CamelCase because data read from Unity created json
string coordinate_system
string agent_type
int32 publish_frequency
float32 image_size_scale
bool is_mono_on
bool is_mono_depth_on
int32 rotation_x_mono
int32 focal_length_mono
int32 clipping_mono
bool is_stereo_on
bool is_stereo_depth_on
bool is_stereo_no_agents_on
bool is_stereo_point_cloud_on
int32 rotation_x_stereo
int32 focal_length_stereo
int32 clipping_stereo
bool is_fisheye_on
bool is_eagleeye_on
Agent[] agents
