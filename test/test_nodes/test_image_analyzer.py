#!/usr/bin/env python
PKG = 'openag_brain'
NAME = 'test_image_analyzer'

import sys
import unittest
import tempfile
import rospy
from std_msgs.msg import Float64

def get_temp_file_name():
    return tempfile.gettempdir() + '/' + next(tempfile._get_candidate_names())

class TestImageAnalyzer(unittest.TestCase):
    """
    Test the imaeg analyzer node. It should read an image from usb_cam and
    process it through the src/openag_cv modules.
    """
    def setUp(self):
        self.namespace = "tests"
        self.image_received = False

    def callback(self, item):
        print("===================Image Received--------------------")
        self.image_received = True
        rospy.loginfo("Image Received {} {}".format(item.width, item.height))
        image_format = self.image_format_mapping.get(item.encoding, None)
        if image_format is None:
            raise ValueError()
        img = Image.fromstring(
            image_format, (item.width, item.height), item.data
        )
        file_name = get_temp_file_name()
        img.save(file_name)
        print("Image saved to ".format(file_name))
        self.assertTrue(item.width == 480)
        self.assertTrue(item.height == 640)

    def test_usb_cam(self):
        rospy.init_node(NAME, anonymous=True)
        camera_source = "test_image"
        self.sub_measured = rospy.Subscriber("{}/{}/image_raw".format(self.namespace, camera_source),
                                            Float64, self.callback)
        for _ in range(15,0,-1):
           rospy.loginfo("Waiting {} seconds to receive image.".format(_))
           rospy.sleep(1)  # Wait for subscriber to be ready to receive messages
        # for msg in self.msgs:
        #     rospy.loginfo("Published: {:f}".format(msg))
        #     self.pub_desired.publish(msg)
        # rospy.sleep(15)  # Wait until all messages are received.
        #rospy.spin()
        self.assertTrue(self.image_received)


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestImageAnalyzer)
