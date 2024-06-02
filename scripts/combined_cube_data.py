#!/usr/bin/env python3

import rospy
import cv2
import os
import numpy as np
import sys
import rospkg
import logging
import ros_numpy
from hyper_drive.msg import DataCube
from std_msgs.msg import Header
from sensor_msgs.msg import Image

class CombineDataCube(object):
    def __init__(self):
        
        # Place holder variable for received images
        self.ximea_cube = []
        self.imec_cube = []
        self.alvium = []
        self.imec_msg = None
        self.ximea_msg = None
        
        # Load homography matrices
        self.ros_pack = rospkg.RosPack()
        # IMEC to Alvium
        self.i2a = np.load(os.path.join(self.ros_pack.get_path('hyper_drive'),'config','homographies','i2a.npy'))
        # XIMEA to Alvium
        self.x2a = np.load(os.path.join(self.ros_pack.get_path('hyper_drive'),'config','homographies','x2a.npy'))
        # Load mask for cimbined data area
        self.mask = np.load(os.path.join(self.ros_pack.get_path('hyper_drive'),'config','homographies','mask.npy'))

        # Subscribe to incoming data values
        rospy.Subscriber('/ximea/undistort_data', DataCube, self.callback_ximea)
        rospy.Subscriber('/imec/undistort_data', DataCube, self.callback_imec)
        rospy.Subscriber('/camera/image_raw', Image, self.callback_alvium)

        # Publish output data
        self.pub_merge = rospy.Publisher('/combined/undistort_data', DataCube, queue_size=10)
        self.pub_pan = rospy.Publisher('/combined/pan', Image, queue_size=10)

    def callback_imec(self, msg: DataCube) -> None:
        '''
        Callback function to receive undistorted IMEC datacube
        '''
        self.imec_msg = msg
        self.imec_cube = np.reshape(msg.data, (msg.width, msg.height, msg.lam))


    def callback_ximea(self, msg: DataCube) -> None:
        '''
        Callback function to receive undistorted XIMEA datacube
        '''
        self.ximea_msg = msg
        self.ximea_cube = np.reshape(msg.data, (msg.width, msg.height, msg.lam))

    
    def callback_alvium(self, msg: Image) -> None:
        '''
        Callback function to receive udistorted RGB image
        '''
        self.alvium = ros_numpy.numpify(msg)


    def warp_and_merge(self, alvium: np.ndarray, ximea: np.ndarray, imec: np.ndarray, apply_filter: bool=False) -> np.ndarray:
        '''
        Create combined hypercube by using affine transformations
        '''
        print(f"ximea: {ximea.shape} alvium: {alvium.shape} imec: {imec.shape}")
        cube = np.zeros((*alvium.shape, ximea.shape[2]+imec.shape[2]))
        # Warp the ximea image
        cube[:,:,:ximea.shape[2]] = cv2.warpAffine(
            ximea,
            self.x2a, 
            (alvium.shape[1], alvium.shape[0]),
        )
        # Warp the imec image:
        cube[:,:,-imec.shape[2]:] = cv2.warpAffine(
            imec,
            self.i2a, 
            (alvium.shape[1], alvium.shape[0]),
        )
        # apply final crop to maximum available data
        return cube
    
    def combine_and_publish(self, out: np.ndarray) -> None:
        '''
        Take datacube, and generate metadata
        '''
        print('Combining')
        toSend = DataCube()
        # Create header
        h = Header()
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        toSend.header = h
        toSend.data = out.flatten()
        toSend.width,toSend.height,toSend.lam = out.shape
        toSend.central_wavelengths = np.concatenate((self.ximea_msg.central_wavelengths, self.imec_msg.central_wavelengths), axis=None)
        toSend.qe = np.concatenate((self.ximea_msg.qe, self.imec_msg.qe), axis=None)
        toSend.fwhm_nm = np.concatenate((self.ximea_msg.fwhm_nm, self.imec_msg.fwhm_nm), axis=None)
        self.pub_merge.publish(toSend)


    def start(self) -> None:
        '''
        Main execution  loop to enable periodic publishing
        of combined hyperspectral datacube
        '''
        while not rospy.is_shutdown():
            if self.ximea_cube != [] and self.imec_cube !=[] and self.alvium != []:
                out = self.warp_and_merge(self.alvium, self.ximea_cube, self.imec_cube)
                self.combine_and_publish(out)


if __name__ == '__main__':
    rospy.init_node('CombineDataCube', anonymous=True)
    my_node = CombineDataCube()
    my_node.start()
