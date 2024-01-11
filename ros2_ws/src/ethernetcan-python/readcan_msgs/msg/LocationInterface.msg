###########################################
### Bosch Radar Gen5 Raw Data Interface ###
###########################################

std_msgs/Header header

# Meta data
float32 radar_ecu_timestamp

# Location data and measurement state
Location[]   raw_locations
SensingState sensing_state
