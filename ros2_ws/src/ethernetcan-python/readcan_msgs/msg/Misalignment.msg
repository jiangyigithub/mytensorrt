#####################################
### Bosch Radar Gen5 Misalignment ###
#####################################

# Flags for estimation_status
uint8 ESTIMATION_STATUS_VALID = 0

uint8 estimation_status

# Azimuth angle
float32 azimuth_angle_misalignment
float32 azimuth_angle_misalignment_variance

# Elevation angle
float32 elevation_angle_misalignment
float32 elevation_angle_misalignment_variance
