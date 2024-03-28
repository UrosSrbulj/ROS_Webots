#!/usr/bin/env python3

import rospy 
from sensor_msgs.msg import Image
from rosbot_webots.camera_filter  import CameraFilter
import numpy as np
from enum import Enum
import time 
import std_msgs.msg as msg 


import math
import matplotlib.pyplot as plt



class CameraNode:
    
    def __init__(self):

        rospy.init_node("camera_filter")
        rospy.loginfo("Starting camera_filter node.")

        self.camera_sub = rospy.Subscriber("webots_camera", Image, callback = self.callback_camera)
        

        print('slikanje')

        self.cf_class = CameraFilter()

    def callback_camera(self, image):
        print('usao u callback')
        self.cf_class.cut_image(image)
        
     


    def run(self):
        rospy.spin()

    
if __name__ == "__main__":
    node = CameraNode()
    node.run()