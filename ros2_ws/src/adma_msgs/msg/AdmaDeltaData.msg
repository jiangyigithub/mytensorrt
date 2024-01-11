uint16  target_gps_mode
uint16  hunter_gps_mode
uint64  ins_gnss_ts_msec_epoch
uint32  ins_time_msec
uint16  ins_time_week
int16   leap_seconds

# -180.0 to +180.0, 
float64 target_longitude
# decimal of Longitude, 
float32 target_longitude_decimal_places

# -90.0 to +90.0,
float64 target_latitude
# decimal of latitude
float32 target_latitude_decimal_places

# Longitude delta distance, in meter
float32 long_delta_distance
# Longitude delta velocity, in m/s
float32 long_delta_velocity

# latitude delta distance, in 0.01 meter
float32 lat_delta_distance
# latitude delta velocity, in 0.01 m/s
float32 lat_delta_velocity

# Resultant distance, in meter
float32 resultant_distance
# Resultant velocity, in m/s
float32 resultant_velocity

#Angle of orientation, -180.0 to +180.0 degree
float32 angle_of_orientation

#Delta time in msec
uint32 delta_time

# ±160.0,  m/s
float32 target_forward_velocity
float32 hunter_forward_velocity
# ±160.0,  m/s²
float32 target_forward_acceleration
float32 hunter_forward_acceleration
# ±160.0,  m/s
float32 target_lateral_velocity
float32 hunter_lateral_velocity
# ±160.0,  m/s²
float32 target_lateral_acceleration
float32 hunter_lateral_acceleration
# ±180.0,  degree
float32 target_pitch_angle
float32 hunter_pitch_angle