
#!/usr/bin/env python3


from cv2 import waitKey
import rospy
from sensor_msgs.msg import Image
import cv2
import cv_bridge as cvb 
from cv_bridge import CvBridgeError

from enum import Enum
import time 
import std_msgs.msg as msg 

import math
import matplotlib.pyplot as plt
import numpy as np



class CameraFilter:

    def __init__(self):
        self.cvb = cvb.CvBridge()

    def cut_image(self, data):
        try:
            cv_image = self.cvb.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e: 
            print(e)

       # cropped = cv_image[start_row:end_row, start_col:end_col]
        cv_image = cv_image[70:790,50:1030]
        cv2.imshow('original image',cv_image)
        cv2.waitKey(1)

        

        hsv = cv2.cvtColor(cv_image ,cv2.COLOR_BGR2HSV)

        donja_granica_zute = np.array([20,50,50])
        gornja_granica_zute = np.array([33,255,255])
        maska_zute_boje = cv2.inRange(hsv, donja_granica_zute,gornja_granica_zute)

        donja_granica_crvene = np.array([0, 50, 50])
        gornja_granica_crvene = np.array([12, 255, 255])
        maska_crvene_boje = cv2.inRange(hsv, donja_granica_crvene,gornja_granica_crvene)

        donja_granica_narandzaste = np.array([10, 30, 90])
        gornja_granica_narandzaste = np.array([150, 240, 251])
        maska_narandzaste_boje = cv2.inRange(hsv, donja_granica_narandzaste,gornja_granica_narandzaste)

        donja_granica_plave = np.array([118, 50, 50])
        gornja_granica_plave = np.array([125, 255, 255])
        maska_plave_boje = cv2.inRange(hsv, donja_granica_plave,gornja_granica_plave)


        #cv2.imshow(' image', maska_narandzaste_boje)
        #Cv2.waitKey(1)
        #rospy.loginfo("odra")

        redovi= maska_zute_boje.shape[0]
        kolone = maska_zute_boje.shape[1]
        matrix_prepreka= np.zeros(maska_zute_boje.shape)
        matrix_stop= np.zeros(maska_crvene_boje.shape)
        matrix_objekat= np.zeros(maska_narandzaste_boje.shape)
        matrix_robot= np.zeros(maska_plave_boje.shape)

        for i in range(redovi):
            for j in range(kolone):
             matrix_prepreka[i][j] = np.where(maska_zute_boje[i][j] ==255,1,0)
             matrix_stop[i][j] = np.where(maska_crvene_boje[i][j] ==255,2,0)
             matrix_objekat[i][j] = np.where(maska_narandzaste_boje[i][j] ==255,3,0)
             matrix_robot[i][j] = np.where(maska_plave_boje[i][j] ==255,4,0)
             

           # print(matrix.shape)
        smanjena_matrica_prepreka = cv2.resize(matrix_prepreka,(36 ,36), interpolation=cv2.INTER_CUBIC)
        smanjena_matrica_stop = cv2.resize(matrix_stop,(36 ,36), interpolation=cv2.INTER_CUBIC)
        smanjena_matrica_objekat = cv2.resize(matrix_objekat,(36 ,36), interpolation=cv2.INTER_CUBIC)
        smanjena_matrica_robot = cv2.resize(matrix_robot,(36 ,36), interpolation=cv2.INTER_CUBIC)
        
        suma_matrica = smanjena_matrica_prepreka+smanjena_matrica_objekat+smanjena_matrica_robot+smanjena_matrica_stop

      

        
        np.savetxt('/home/uros/Documents/matricaa_prepreka.txt', smanjena_matrica_prepreka, fmt = '%.2d')
        np.savetxt('/home/uros/Documents/matricaa_stop.txt', smanjena_matrica_stop, fmt = '%.2d')
        np.savetxt('/home/uros/Documents/matricaa_objekat.txt', smanjena_matrica_objekat, fmt = '%.2d')
        np.savetxt('/home/uros/Documents/matricaa_robot.txt', smanjena_matrica_robot, fmt = '%.2d')
      #  np.savetxt('/home/uros/Documents/suma_matrica.txt', suma_matrica, fmt = '%.2d')

        rospy.set_param("/suma_matrica", suma_matrica.tolist())
    
    



      
