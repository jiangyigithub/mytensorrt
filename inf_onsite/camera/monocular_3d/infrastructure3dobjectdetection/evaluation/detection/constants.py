# Based on nuScenes dev-kit written by Oscar Beijbom and Varun Bankiti, 2019.

# DETECTION_NAMES = ['car', 'van', 'truck', 'trailer','bus', 'pedestrian',
# 'cyclist', 'motorcyclist', 'barrow' ,'tricyclist']
DETECTION_NAMES = [
    'car', 'van', 'truck', 'bus', 'pedestrian', 'cyclist', 'motorcyclist',
    'tricyclist', 'barrow'
]

# DETECTION_NAMES = ['car', 'truck', 'bus', 'trailer', 'construction_vehicle',
# 'pedestrian', 'motorcycle', 'bicycle','traffic_cone', 'barrier']

# PRETTY_DETECTION_NAMES = {'car': 'Car',
#                           'truck': 'Truck',
#                           'bus': 'Bus',
#                           'trailer': 'Trailer',
#                           'construction_vehicle': 'Constr. Veh.',
#                           'pedestrian': 'Pedestrian',
#                           'motorcycle': 'Motorcycle',
#                           'bicycle': 'Bicycle',
#                           'traffic_cone': 'Traffic Cone',
#                           'barrier': 'Barrier'}

PRETTY_DETECTION_NAMES = {
    'car': 'car',
    'truck': 'truck',
    'bus': 'bus',
    'van': 'van',
    # 'trailer': 'trailer',
    'cyclist': 'cyclist',
    'motorcyclist': 'motorcyclist',
    #   'construction_vehicle': 'Constr. Veh.',
    'pedestrian': 'pedestrian',
    'motorcycle': 'motorcycle',
    # 'Bicycle': 'Bicycle',
    'tricyclist': 'tricyclist',
    #   'traffic_cone': 'Traffic Cone',
    'barrow': 'barrow'
}

# DETECTION_COLORS = {'car': 'C0',
#                     'truck': 'C1',
#                     'bus': 'C2',
#                     'trailer': 'C3',
#                     'construction_vehicle': 'C4',
#                     'pedestrian': 'C5',
#                     'motorcycle': 'C6',
#                     'bicycle': 'C7',
#                     'traffic_cone': 'C8',
#                     'barrier': 'C9'}

DETECTION_COLORS = {
    'car': 'C0',
    'truck': 'C1',
    'bus': 'C2',
    'van': 'C3',
    'trailer': 'C4',
    'cyclist': 'C5',
    'motorcyclist': 'C6',
    #   'construction_vehicle': 'Constr. Veh.',
    'pedestrian': 'C7',
    'motorcycle': 'C8',
    'bicycle': 'C9',
    'tricyclist': 'C10',
    #   'traffic_cone': 'Traffic Cone',
    'barrow': 'C11'
}

ATTRIBUTE_NAMES = [
    'pedestrian.moving', 'pedestrian.sitting_lying_down',
    'pedestrian.standing', 'cycle.with_rider', 'cycle.without_rider',
    'vehicle.moving', 'vehicle.parked', 'vehicle.stopped'
]

PRETTY_ATTRIBUTE_NAMES = {
    'pedestrian.moving': 'Ped. Moving',
    'pedestrian.sitting_lying_down': 'Ped. Sitting',
    'pedestrian.standing': 'Ped. Standing',
    'cycle.with_rider': 'Cycle w/ Rider',
    'cycle.without_rider': 'Cycle w/o Rider',
    'vehicle.moving': 'Veh. Moving',
    'vehicle.parked': 'Veh. Parked',
    'vehicle.stopped': 'Veh. Stopped'
}

# TP_METRICS = ['trans_err', 'scale_err', 'orient_err', 'vel_err', 'attr_err']
TP_METRICS = ['trans_err', 'scale_err']

PRETTY_TP_METRICS = {
    'trans_err': 'Trans.',
    'scale_err': 'Scale',
    'orient_err': 'Orient.',
    'vel_err': 'Vel.',
    'attr_err': 'Attr.'
}

TP_METRICS_UNITS = {
    'trans_err': 'm',
    'scale_err': '1-IOU',
    'orient_err': 'rad.',
    'vel_err': 'm/s',
    'attr_err': '1-acc.'
}
