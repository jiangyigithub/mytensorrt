#!/usr/bin/env python3
""" Nosetests, testing the python interface"""

from geometry_msgs.msg import Vector3, TransformStamped
from perception_kit_msgs.msg import Object
import perception_kit_object_transform

def test_identity_transform():
    """ testing the python interface by transforming an object with the identity transform"""
    obj = Object()
    obj.header.frame_id = "test_01"

    transform = TransformStamped()
    transform.header.frame_id = "test_02"
    transform.child_frame_id = "test_01"
    transform.transform.rotation.w = 1
    velocity = Vector3()
    acceleration = Vector3()

    ret = perception_kit_object_transform.transformObject(obj, transform, velocity, acceleration)

    assert ret.header.frame_id == "test_02"
