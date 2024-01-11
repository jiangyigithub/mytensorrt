# Radial distance [m]
float32 radial_distance
# Radial distance variance [m^2]
float32 radial_distance_variance
# Relative radial velocity [m/s]
float32 radial_velocity
# Variance of radial velocity [m^2/s^2]
float32 radial_velocity_variance

# Covariance of radial distance and velocity [m^2/s]
float32 radial_distance_velocity_covariance

# Azimuth angle (theta) [rad]
float32 azimuth_angle
# Variance of azimuth angles [rad^2]
float32 azimuth_angle_variance
# 1 - azimuth angle quality from location
float32 azimuth_false_detection_rate
# Azimuth turn rate (theta) [rad/s] (currently always 0)
float32 azimuth_turn_rate
# Azimuth turn rate variance [rad^2/s^2] (currently always max float)
float32 azimuth_turn_rate_variance

# Elevation angle (phi) [rad]
float32 elevation_angle
# 1 - elevation angle quality from location
float32 elevation_false_detection_rate

# Radar cross section [dBm2]
float32 rcs

# threshold on azimuth angle quality
bool is_valid
float32 false_detection_rate
float32 true_detection_rate

uint32 id
