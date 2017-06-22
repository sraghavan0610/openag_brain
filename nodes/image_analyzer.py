#!/usr/bin/env python

import time
import rospy
import requests
from PIL import Image
from couchdb import Server
from StringIO import StringIO
from sensor_msgs.msg import Image as ImageMsg
from re import match

from openag.cli.config import config as cli_config
from openag.models import EnvironmentalDataPoint, SoftwareModule
from openag.db_names import ENVIRONMENTAL_DATA_POINT, SOFTWARE_MODULE
from openag_brain.load_env_var_types import create_variables
from openag_brain import params
from openag_brain.utils import read_environment_from_ns

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

    def on_image(self, item):
        # Rate limit
        curr_time = time.time()
        if (curr_time - self.last_update) < self.min_update_interval:
            return
        self.last_update = curr_time

        rospy.loginfo("Posting image")
        print("Image Received!!!!")
        # image_format = self.image_format_mapping.get(item.encoding, None)
        # if image_format is None:
        #     raise ValueError()
        # img = Image.fromstring(
        #     image_format, (item.width, item.height), item.data
        # )
        # img.save('/home/pi/images/')
