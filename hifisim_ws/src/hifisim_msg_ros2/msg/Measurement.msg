int32 sensor_id
int32 time
int32 frame_id
int32 class_id		# set to -1 if there is no detection
float32 confidence	# set to 0 if there is no detection
float32 xcenter
float32 ycenter
float32 zcenter
float32 box_width
float32 box_height
geometry_msgs/Point[4] fov_corners   # First point at top left, go clockwise
float32 xgroundtruth
float32 ygroundtruth
