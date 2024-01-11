#!/usr/bin/env python3
import argparse
import rclpy
#import rospy
import rosunit
import sys
import unittest
from perception_kit_msgs.msg import Objects, Object, Motion


@unittest.skipIf( '--skip' in sys.argv, "skip requested")
class OutputValidator(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(OutputValidator, self).__init__(*args, **kwargs)

        self.received_data = False
        self.test_good = True
        self.error_message = ""
        self.counter = 0
        self.max_messages = 10

        self.objects_publisher = self.create_publisher(Objects, "/fusion/obstacles", queue_size=10)
        #self.objects_publisher = rospy.Publisher('/fusion/obstacles', Objects , queue_size=10)
        self.objects_publisher = self.create_publisher(Motion, "/egoestimation/odometry_referenced_egomotion", queue_size=10)
        #self.motion_publisher = rospy.Publisher('/egoestimation/odometry_referenced_egomotion', Motion , queue_size=10)

        self.output_subscriber = self.create_subscription(Objects, "/perception_kit/fusion/objects", self.validate)
        #self.output_subscriber = rospy.Subscriber("/perception_kit/fusion/objects", Objects, self.validate)

    def runTest(self):
        OutputValidator('test_validate_output').test_validate_output()

    def validate(self,msg):
        self.get_logger().info("Got data")
        #rospy.logdebug("Got data")
        self.counter = self.counter + 1
        self.assertTrue(msg.header.frame_id == "base_link")
        self.test_good = True
        self.received_data = True

    def publish_egomotion(self):
        msg = Motion()
        msg.header.stamp = self.get_clock().now() #rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.motion_frame_id = "base_link"
        msg.twist.twist.linear.x = 10

        self.motion_publisher.publish(msg)

    def publish_objects(self):
        msg = Objects()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now() #rospy.Time.now()

        o = Object()
        o.header.frame_id = "odom"
        o.header.stamp = self.get_clock().now() #rospy.Time.now()
        o.velocity.x = 3

        msg.objects.append(o)

        self.objects_publisher.publish(msg)

    def test_validate_output(self):

        rate = rclpy.Rate(10)
        #current_time = rospy.get_rostime().to_sec()
        while not rclpy.is_shutdown() and self.test_good and self.counter < self.max_messages:
            
            self.publish_egomotion()
            self.publish_objects()

            rate.sleep()
            #current_time = rospy.get_rostime().to_sec()
            self.get_logger().info("Msgs remaining: {}".format(self.max_messages - self.counter))
            #rospy.loginfo("Msgs remaining: {}".format(self.max_messages - self.counter))

        self.assertTrue(self.test_good, self.error_message)
        self.assertTrue(self.received_data,"{}: Never received data ".format(sys.argv[1]))

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('test_name', type=str, help='test name')
    parser.add_argument('--skip', dest='skip', action='store_true',help='skips this unit test')
    parser.add_argument('--text', dest='text', action='store_true',help='makes the unit test output log messages')
    args , unknown = parser.parse_known_args()

    #rospy.init_node('perception_kit_object_transform_test', anonymous=True , log_level=rospy.DEBUG )
    rclpy.create_node('perception_kit_object_transform_test')

    rosunit.unitrun('difusion_output_validation', "{}_".format(args.test_name) , OutputValidator , sys.argv )
