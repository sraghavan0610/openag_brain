#!/usr/bin/env python
PKG = 'openag_brain'
NAME = 'test_image_analyze'

import sys
import unittest
import tempfile
import rospy
from std_msgs.msg import Float64

def get_temp_file_name():
    return tempfile.gettempdir() + '/' + next(tempfile._get_candidate_names())

class TestImageAnalyze(unittest.TestCase):
    """
    Test the imaeg analyzer node. It should read an image from usb_cam and
    process it through the src/openag_cv modules.
    """
    def setUp(self):
        self.namespace = "tests"

    def callback(self, item):
        rospy.loginfo("Image Received {} {}".format(item.width, item.height))
        # self.assertTrue()
        image_format = self.image_format_mapping.get(item.encoding, None)
        if image_format is None:
            raise ValueError()
        img = Image.fromstring(
            image_format, (item.width, item.height), item.data
        )
        img.save(get_temp_file_name())
        
    def test_usb_cam(self):
        rospy.init_node(NAME, anonymous=True)
        camera_source = "aerial_image"
        self.sub_measured = rospy.Subscriber("{}/{}/image_raw".format(self.namespace, camera_source),
                                            Float64, self.callback)
        # rospy.sleep(15)  # Wait for subscriber to be ready to receive messages
        # for msg in self.msgs:
        #     rospy.loginfo("Published: {:f}".format(msg))
        #     self.pub_desired.publish(msg)
        # rospy.sleep(15)  # Wait until all messages are received.

        self.assertTrue(self.msgs == self._received)


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestImageAnalyze)
