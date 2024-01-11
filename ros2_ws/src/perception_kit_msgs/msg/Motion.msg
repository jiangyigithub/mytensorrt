std_msgs/Header header

# Pose, relative to the frame_id specified in the header
geometry_msgs/PoseWithCovariance pose

# Twist and acceleration relative to the motion frame
string motion_frame_id
geometry_msgs/TwistWithCovariance twist
geometry_msgs/AccelWithCovariance accel

# Curvature and its derivation
float32 curvature
float32 dcurvature
