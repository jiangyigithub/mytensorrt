uint16  status_gps_mode
uint64  ins_gnss_ts_msec_epoch
uint32  ins_time_msec
uint16  ins_time_week
int16   leap_seconds

float64 ins_lat_abs
float64 ins_long_abs
float32 ins_height

float32 ins_roll
float32 ins_pitch
float32 ins_yaw
float32 gps_cog
float32 gps_cog_in_enu

# NED coordinate system
# X axis points positive to the north
float32 ins_vel_frame_x
#Y axis points positive to the east
float32 ins_vel_frame_y
#Z axis points positive down
float32 ins_vel_frame_z

#The axes of the horizontal coordinate system are aligned in such a way that the XY axis level is orthogonal 
#to the acceleration of gravity vector (in the horizon),
float32 ins_vel_hor_x
float32 ins_vel_hor_y
float32 ins_vel_hor_z

#The axes of the horizontal coordinate system are aligned in such a way that the XY axis level is orthogonal 
#to the acceleration of gravity vector (in the horizon),
float32 acc_hor_x
float32 acc_hor_y
float32 acc_hor_z



