#!/usr/bin/env python

import os
import numpy as np
import cv2

import rospy 
import rospkg
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import String

br = CvBridge()
ros_root = rospkg.get_ros_root()
r = rospkg.RosPack()

rubix_solver_path = r.get_path('rubix_cube_robot_solver')
img_path = os.path.join(rubix_solver_path, "test_image")
# print(img_path)

class AVT_Hallucination(object):
    """
        Hallucination for rubix cube face image.
    """
    def __init__(self, path=img_path):

        rospy.init_node('triggered_avt_camera', anonymous=True)
        self.load_image(path)
        self.pub_init()
        self.sub_init()
        rospy.spin()

    def pub_init(self):
        self.pub = rospy.Publisher("avt_camera_img", Image, queue_size=1)
        
    def sub_init(self):
        # self.stateMachine = 0
        self.sub = rospy.Subscriber("trigger", String, self.trigger_callback)

    def load_image(self, path):
        self.img_buffer = {}
        img_list = os.listdir(path)
        for img_name in img_list:
            face_name = img_name[0]
            img = cv2.imread(os.path.join(path, img_name), 0)
            # img = cv2.cvtColor()
            self.img_buffer[face_name] = img
        # print("length of image buffer = ", len(self.img_buffer))
        print("load image finish")

    def trigger_callback(self, msg):
        # if stateMachine:
        # print(msg.data)
        img = self.getImage(msg.data)
        image_msg = br.cv2_to_imgmsg(img)
        self.pub.publish(image_msg)

    def getImage(self, msg, mode=0):
        if mode == 0:
            return self.img_buffer[msg]
        # if mode == 1:
        #     seq = 

if __name__ == "__main__":
    avt_hal = AVT_Hallucination(img_path)






