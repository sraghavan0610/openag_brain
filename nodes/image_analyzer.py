#!/usr/bin/env python
"""
The `image_analyzer.py` module listens for image data from an environment,
and processes the image for the openag_cv modules. The new process images and any
relevant data are then published to a new ros topic for analysis.

It assumes all topics of the type `sensor_msgs/Image` under the namespace
for the environment are streams of images from connected webcams.

TODO: Save a list of openag_cv modules to use in the param server so they can be turned on and off as desired
"""
import rospy
from PIL import Image
from sensor_msgs.msg import Image as ImageMsg
from openag_brain.load_env_var_types import create_variables
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Filter a list of environmental variables that are specific to camera
CAMERA_VARIABLES = create_variables(rospy.get_param('/var_types/camera_variables'))

class ImageProcess:
    image_format_mapping = {
        "rgb8": "RGB",
        "rgba8": "RGBA"
    }

    def __init__(self, db, topic, variable, environment, min_update_interval):
        self.db = db
        self.variable = variable
        self.environment = environment
        self.min_update_interval = min_update_interval
        self.last_update = 0
        self.sub = rospy.Subscriber(topic, ImageMsg, self.on_image)


image_format_mapping = {
    "rgb8": "RGB",
    "rgba8": "RGBA"
}
bridge = CvBridge()

def on_image(self, item):
    rospy.loginfo("===================Image Received===================")
    self.image_received = True
    rospy.loginfo("Image Received {} {}".format(item.width, item.height))
    image_format = self.image_format_mapping.get(item.encoding, None)
    if image_format is None:
        raise ValueError()
    cv2_img = self.bridge.imgmsg_to_cv2(item, "bgr8")
    #np_arr = np.fromstring(item.data, np.uint8)
    #image_np = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
    file_name = get_temp_file_name()
    rospy.loginfo("Saving image to {}".format(file_name))
    cv2.imwrite(file_name, cv2_img)
    # rospy.loginfo("Image saved to {}".format(file_name))
    rospy.loginfo("width: {} height: {}".format(item.width, item.height))
    self.assertTrue(item.width == 640)
    self.assertTrue(item.height == 480)
    self.image_processed = True

def main():
    image_received = False
    rospy.init_node(NAME, anonymous=True)
    #camera_source = "test_image"
    image_rostopic = "{}/image_raw".format(self.namespace)
    sub_measured = rospy.Subscriber(image_rostopic, Image, self.callback)
    rospy.loginfo("Looking for image at " + image_rostopic)

