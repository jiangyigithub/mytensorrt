#!/usr/bin/env python
"""
Examines the track lifetimes of dynamic radar objects in a bagfile.
Generally, a lower average track lifetime is worse than a higher average.
This script can be used as evaluation metric of ts2 tests.
"""

import rosbag
import math
import numpy as np
from ts2.result import Result
from ts2.metric import Metric


def object_lifetimes(bag_to_verify, obj_pexist_thresh):
    """
    extract the lifetimes of all unique objects tracked within the bag
    """
    last_t = 0.0
    dt = 0.0
    finished_object_times = []  # List of durations of intividual objects
    current_object_times = {}  # Dictionary of objects with durations currently tracked
    for topic, msg, t in bag_to_verify.read_messages(topics=['/perception/radar_gen5/tracks']):
        current_keys = []  # the keys we see in this message
        t_secs = float(msg.header.stamp.secs) + 1e-9 * \
            float(msg.header.stamp.nsecs)
        if last_t == 0:
            last_t = t_secs
        elif t_secs > last_t:
            dt = t_secs - last_t
            last_t = t_secs

        for obj in msg.objects:
            # Check whether the object meets existence criteria
            if obj.existence_probability > obj_pexist_thresh:
                # Check if this object ID is in our list of current ones being tracked
                if obj.id not in current_keys:
                    # Add the current object key to those we have seen in this message
                    current_keys.append(obj.id)
                # Check if we are currently tracking this object
                if obj.id in current_object_times.keys():
                    # extend the tracked time of this object
                    current_object_times[obj.id] += dt
                else:
                    # add this object to track the time
                    current_object_times[obj.id] = 0

        # Now check if there are any object tracks that were lost
        lost_object_ids = np.setdiff1d(
            current_object_times.keys(), current_keys)
        for idx in lost_object_ids:
            # idx was lost. Add its total time to the list
            finished_object_times.append(current_object_times[idx])
            # Delete the object from the currently tracked list
            current_object_times.pop(idx)

    # Add the final times to the finished object times
    for idx in current_object_times.keys():
        # idx was lost. Add its total time to the list
        finished_object_times.append(current_object_times[idx])
        # Delete the object from the currently tracked list
        current_object_times.pop(idx)

    return finished_object_times


if __name__ == '__main__':

    m = Metric()
    recording_bag = m.get('recording_bag', default='recording.bag')
    obj_pexist_thresh = m.get('obj_pexist_thresh')
    expected_lifetime = m.get('expected_lifetime')
    long_track_lifetime_thresh = m.get('long_track_lifetime_thresh')
    expected_long_track_lifetime = m.get('expected_long_track_lifetime')
    expected_short_tracks_count = m.get('expected_short_tracks_count')
    print "object p_exist threshold: " + str(obj_pexist_thresh) + "."
    print "expected track lifetime: " + str(expected_lifetime) + "."
    print "long track lifetime threshold: " + \
        str(long_track_lifetime_thresh) + "."
    print "expected track lifetime: " + str(expected_lifetime) + "."
    print "expected short tracks count: " + \
        str(expected_short_tracks_count) + "."

    # Open the bag
    bag = rosbag.Bag(recording_bag)

    # Default to success
    exit_code = 0

    # Sanity check for empty bag
    num_msgs = bag.get_message_count('/perception/radar_gen5/tracks')
    result1 = Result(expected=1, actual=num_msgs,
                     save_as="track_message_count")
    if num_msgs == 0:
        result1.fail("No messages in bag.")
        exit_code = 1

    # Determine average object lifetime. Longer is better as they generally reflect more consistent tracking and less
    # short-lived ghost objects
    lt = object_lifetimes(bag, obj_pexist_thresh)

    # convert to numpy array
    lifetimes = np.array(lt)

    # Look at the average lifetime of all tracks
    average_lifetime_all = float(np.mean(lifetimes))
    result2 = Result(expected=expected_lifetime,
                     actual=average_lifetime_all, save_as="average_object_lifetime")
    result2_str = "Average track lifetime: " + \
        str(average_lifetime_all) + ", min " + \
        str(expected_lifetime) + " allowed."
    if expected_lifetime < average_lifetime_all:
        result2.succeed(result2_str)
    else:
        result2.fail(result2_str)
        exit_code = 1

    # Now threshold to look at long tracks. The threshold should be set to remove all short-lived objects
    average_lifetime_long = float(np.mean(
        lifetimes[lifetimes > long_track_lifetime_thresh]))
    result3 = Result(expected=expected_long_track_lifetime,
                     actual=average_lifetime_long, save_as="average_object_lifetime_long")
    result3_str = "Average long track lifetime: " + \
        str(average_lifetime_long) + ", min " + \
        str(expected_long_track_lifetime) + " allowed."
    if expected_long_track_lifetime < average_lifetime_long:
        result3.succeed(result3_str)
    else:
        result3.fail(result3_str)
        exit_code = 1

    # Now count how many short tracks there are
    short_tracks_count = lifetimes[lifetimes <=
                                   long_track_lifetime_thresh].shape[0]
    result4 = Result(expected=expected_short_tracks_count,
                     actual=short_tracks_count, save_as="short_tracks_count")
    result4_str = "Number of short lifetime tracks: " + \
        str(short_tracks_count) + ", max " + \
        str(expected_short_tracks_count) + " allowed."
    if expected_short_tracks_count > short_tracks_count:
        result4.succeed(result4_str)
    else:
        result4.fail(result4_str)
        exit_code = 1
    bag.close()
    exit(exit_code)
