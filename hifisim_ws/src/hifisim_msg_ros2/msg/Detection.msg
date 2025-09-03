int32 frame_id
float32 frame_width
float32 frame_height
float32 focal_length
float32 horizontal_fov
float32 vertical_fov
geometry_msgs/Pose uav_pose
geometry_msgs/Pose camera_pose
int32 class_id      # set to -1 if there is no detection
string class_str
float32 confidence  # set to 0 if there is no detection
float32 xcenter
float32 ycenter
float32 box_width
float32 box_height
float32 xgroundtruth
float32 ygroundtruth
