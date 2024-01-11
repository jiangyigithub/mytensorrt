# Copyright (c) 2009, 2018 Robert Bosch GmbH and its subsidiaries. This program and the accompanying materials are made
# available under the terms of the Bosch Internal Open Source License v4 which accompanies this distribution, and is
# available at http://bios.intranet.bosch.com/bioslv4.txt
from libperception_kit_object_transform_python import *

from geometry_msgs.msg import TransformStamped, Vector3
from perception_kit_msgs.msg import Object
import rospy
from StringIO import StringIO

def transformObject(o, t, v = Vector3() , a = Vector3()):
    def _to_cpp(msg):
        buf = StringIO()
        msg.serialize(buf)
        return buf.getvalue()

    def _from_cpp(str_msg, cls):
        msg = cls()
        return msg.deserialize(str_msg)

    if not isinstance(o, Object):
            rospy.ROSException('Argument 1 is not a perception_kit_msgs/Object')
    if not isinstance(t, TransformStamped):
            rospy.ROSException('Argument 2 is not a geometry_msgs/TransformStamped')
    if not isinstance(v, Vector3):
            rospy.ROSException('Argument 3 is not a geometry_msgs/Vector3')
    if not isinstance(a, Vector3):
            rospy.ROSException('Argument 4 is not a geometry_msgs/Vector3')

    str_o = _to_cpp(o)
    str_t = _to_cpp(t)
    str_v = _to_cpp(v)
    str_a = _to_cpp(a)

    str_ret = transformObjectWrapper(str_o, str_t, str_v, str_a)
    return _from_cpp(str_ret, Object)
