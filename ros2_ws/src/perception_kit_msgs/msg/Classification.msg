# Object class name
# A string definition of object classes allows for arbitrary names.
# In order to allow exchange with different algorithms working on
# similar topics, we suggest to limit obj_class values to
# english names, lower case, numbers and underscores only.
# A corresponding regular expression is [a-z0-9_].

# In particular, we suggest the following class names:
# animal, bicycle, bridge, building, bus, bus_segment, car,
# car_trailer, caravan, carried_object, construction, e_scooter,
# ego_vehicle, fence, front_of_a_semi_truck, ground, guard_rail,
# license_plate, motorcycle, parking, pedestrian, pickup, pole,
# polegroup, rail_track, rider, road, sidewalk, sky, terrain,
# traffic_light, traffic_sign, train, truck, truck_trailer, tunnel,
# unknown, van, vegetation, wall

# Please be aware of synonyms and related names (e.g. car/vehicle,
# person/pedestrian).
string obj_class

# Object class confidence
# Publishers shall define and ensure consistent values. A common, simple
# definition is:
# Mutually exclusive classes
# - the set of obj_class values is fixed to known classes,
# e.g. {car, pedestrian, unknown}
# - single confidence values are all in the range 0.0 .. 1.0
# - the sum of all confidence values is 1.0
#
# Another variant is to introduce a class-hierarchy,
# e.g. {pedestrian, standing, walking, unknown}. This requires defining
# the hierarchy itself and also how child class values are to be interpreted
# (relative to their parent or absolutely).
float32 confidence

# If there is no custom definition of obj_class and confidence values by
# the publisher, subscribers can assume the following:
# - All obj_class values that will ever occur are used in every message,
#  potentially with a confidence value of 0.0
# - single confidence values are all in the range 0.0 .. 1.0
# - the sum of all confidence values is 1.0
# Publishers that want to use a different definition have to document
# this. This definition is not part of the message interface.
