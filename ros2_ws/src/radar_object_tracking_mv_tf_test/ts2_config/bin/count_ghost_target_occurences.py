#!/usr/bin/env python
"""
Checks wether dynamic objects are present in a bagfile, which are not supposed 
to be in a certain area (defined by radius). These objects are assumed to be 
ghost objects from ground reflexes.
This script can be used as evaluation metric of ts2 tests.
"""

import rosbag
import math
from ts2.result import Result
from ts2.metric import Metric


def count_ground_reflex_ghosts(bag_to_verify, range_to_count_objects, obj_pexist_thresh):
    """
    count radar ghost objects which are unexpected (caused by ground reflexes)
    """
    last_t = 0.0
    dt = 0.0
    total_area_time_false_alarms = 0.0
    
    for topic, msg, t in bag_to_verify.read_messages(topics=['/perception/radar_gen5/tracks']):
        t_secs =  float(msg.header.stamp.secs) + 1e-9 * float(msg.header.stamp.nsecs)
        if last_t == 0:
            last_t = t_secs
        elif t_secs > last_t:
            dt = t_secs - last_t
            last_t = t_secs
            
        for obj in msg.objects:
            if math.sqrt(obj.position.x**2 + obj.position.y**2) < range_to_count_objects:
                if obj.existence_probability > obj_pexist_thresh:
                    total_area_time_false_alarms += obj.length * obj.width * dt 
                    if dt == 0:
                        print "WARNING: dt == 0 when counting number of objects. Something went wrong."

    return total_area_time_false_alarms


if __name__ == '__main__':

    m = Metric()
    recording_bag = m.get('recording_bag', default='recording.bag')
    free_regions_range = m.get('free_regions_range')
    obj_pexist_thresh = m.get('obj_pexist_thresh')
    expected_fa = m.get('expected_fa')

    print "Expected range not to contain radar targets: " + str(free_regions_range) + " m."
    print "object p_exist threshold: " + str(obj_pexist_thresh) + " ."

    exit_code = 0

    bag = rosbag.Bag(recording_bag)

    # Sanity check for empty bag
    num_msgs = bag.get_message_count('/perception/radar_gen5/tracks')
    result1 = Result(expected=1, actual=num_msgs, save_as="track_message_count" )
    if num_msgs == 0:
        result1.fail("No messages in bag.")
        exit_code = 1

    # Determine metric for occurance of ghost objects
    total_area_time_false_alarms = count_ground_reflex_ghosts(bag, free_regions_range, obj_pexist_thresh)
    
    result2 = Result(expected=expected_fa, actual=total_area_time_false_alarms, save_as="ghost_objects_ground_clutter" )
    
    if expected_fa > total_area_time_false_alarms:
        result2.succeed(" " + str(total_area_time_false_alarms) + " targets detected (units: m^2*s), " + str(expected_fa) + " allowed.")
    else:
        result2.fail( " " + str(total_area_time_false_alarms) + " targets detected (units: m^2*s), " + str(expected_fa) + " allowed.")
        exit_code = 1

    bag.close()
    exit(exit_code)
