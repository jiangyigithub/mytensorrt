std_msgs/Header header

int64                                id
# existence probability (lowest:0.0, highest:1.0)
float32                              existence_probability

### Shape
# The bounding box is relative to the position point and yaw, such that the bounding box
# in local object coordinates is
# x: [-length/2, length/2], y: [-width/2, width/2], z: [-height/2, height/2]
float32                              width
float32                              width_variance
float32                              length
float32                              length_variance
float32                              height
float32                              height_variance

# Offset in local object coordinates from the center point (field 'position') to the
# tracked point
float32                              x_offset

# Classification
Classification[]                     classification

### Dynamics / Tracking
# The position of the local frame of the object with regard to the header frame
geometry_msgs/Point                  position
geometry_msgs/Vector3                velocity
geometry_msgs/Vector3                acceleration
float32                              yaw  # aka heading
float32                              yaw_rate

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
uint8 ELEMENT_YAW = 9
uint8 ELEMENT_YAW_RATE = 10
uint8 NUM_COVARIANCE_ELEMENTS = 11

float32[121]                         covariance

# Further attributes
Attribute[] attributes
