#!/usr/bin/python
import os
import argparse
import cv2
import numpy as np
#import rospy


def read_directory(directory_name):
    trans=np.array([[ -1.84738113e-01,-8.71208145e-01,6.85689094e+02],
	           [ -9.99200722e-14,-2.17538575e+00,1.30710154e+03],
	           [ -8.28330460e-17,-1.72667157e-03,1.00000000e+00]],dtype='double') 
    for filename in os.listdir(r"./"+directory_name):
        print(filename) 
        image = cv2.imread(directory_name + "/" + filename)
        dst_img = cv2.warpPerspective(image, trans, (1000, 1200))
        cv2.imwrite("/home/zxkj/Desktop/fu/dst/" + filename, dst_img)


if __name__ == '__main__':
    read_directory("src")
  
    
    
