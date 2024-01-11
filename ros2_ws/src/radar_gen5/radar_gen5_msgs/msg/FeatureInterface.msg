##########################################
### Bosch Radar Gen5 Feature Interface ###
##########################################

std_msgs/Header header

# Meta data
float32 radar_ecu_timestamp
uint32 id
bool is_valid

# radar features (preprocessed locations)
Feature[]   features
