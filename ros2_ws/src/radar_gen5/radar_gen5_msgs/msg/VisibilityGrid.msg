# sidelenght [cells] of (quadratic) grid
int32 grid_size

# cell size in m
float64 cell_size

# vehicle pose in grid
# vehicle pos [cells]
# x in [0, grid_size]
# y in [0, grid_size]
float32 x
float32 y
# vehicle heading angle [rad]
float64 yaw

# Grid (count of grid cells = grid_size * grid_size)
# Visibility value of grid cell denotes the probability at which a dynamic obstacle that is at the
# given position is detected and provided by the DynamicWorld.
# 0 means not visible, 255 means visible
# visibility(x_in_meters, y_in_meters) = visibility[floor(y_in_meters * cell_size) + floor(x_in_meters * cell_size) * grid_size]
uint8[] visibility
