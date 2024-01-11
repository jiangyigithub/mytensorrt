int64                      id

# The position of the local frame of the OBB wrt/ the parent frame [currently vehicle_origin]
geometry_msgs/Point        position

# The velocity of the local frame of the OBB wrt/ the parent frame frozen on the ground [vehicle origin with speed zero] 
# ==> gives object velocities over ground.
geometry_msgs/Vector3      velocity

# The acceleration of the local frame of the OBB wrt/ the parent frame frozen on the ground [vehicle origin with acceleration zero] 
# ==> gives object velocities over ground.
geometry_msgs/Vector3      acceleration

float32                    heading # aka yaw, theta
float32                    yaw_rate

# The bounding box is relative to the position point and heading, such that the bounding box
# is in x: [-length/2+x_offset, length/2+x_offset], y: [-width/2, width/2], z: [0, height]
float32                    length
float32                    width
float32                    height

# Covariance contains all state components (position, ..., height) stacked in the order they are listed above.
# It is represented as a flattened square matrix
# Note: Dimensions of states that are not estimated are filled with zero mean VARIANCE_OF_UNKNOWN variance
# (diagonal element) in the covariance. Correlations (off-diagonal elements) that are not estimated can be set
# to zero.
float64 VARIANCE_OF_UNKNOWN = 100
float64[196]               covariance

# Enumeration of states, as can be used for the covariance matrix when filled into a square matrix
uint8 ELEMENT_POSITION_X = 0
uint8 ELEMENT_POSITION_Y = 1
uint8 ELEMENT_POSITION_Z = 2
uint8 ELEMENT_VELOCITY_X = 3
uint8 ELEMENT_VELOCITY_Y = 4
uint8 ELEMENT_VELOCITY_Z = 5
uint8 ELEMENT_ACCELERATION_X = 6
uint8 ELEMENT_ACCELERATION_Y = 7
uint8 ELEMENT_ACCELERATION_Z = 8
uint8 ELEMENT_HEADING = 9
uint8 ELEMENT_YAW_RATE = 10
uint8 ELEMENT_LENGTH = 11
uint8 ELEMENT_WIDTH = 12
uint8 ELEMENT_HEIGHT = 13
uint8 NUM_COVARIANCE_ELEMENTS = 14

# The distance between the the origin of the local frame of the OBB and the OBB center.
# Positive values denote the origin of the local frame of the OBB behind the OBB center,
# which applies to most normal front-steered vehicles.
float32 x_offset

# existance probability (lowest:0.0, highest:1.0)
float32 existence_probability

# classification of obstacle type
# each probability is independent with a prior of 0.5
float32 pedestrian_probability
float32 vehicle_probability #including motorcycle
float32 bicycle_probability

# classification of intrinsic state of obstacle
# each probability is independent with a prior of 0.5
float32 indicator_left_probability
float32 indicator_right_probability
float32 brake_light_probability
float32 emergency_light_probability

