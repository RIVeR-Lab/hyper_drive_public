#!/usr/bin/env python3

import os
import sys
import cv2 as cv
import rospy
import typing
import rospkg
import numpy as np
import scipy.io as sio
from hyper_drive.msg import DataCube
from std_msgs.msg import String, Header

class CubeCorrecter(object):
    def __init__(self):
        # Setup callback for data
        self.model = rospy.get_param('~camera_model')
        # Load camera distortion parameters
        self.new_camera_matrix = np.array([])
        self.load_distortion_parameters()
        self.pub = rospy.Publisher(f'/{self.model}/undistort_data', DataCube, queue_size=10)
        # Subscribe to the raw data image
        self.sub = rospy.Subscriber(f'/{self.model}/cube_data', DataCube, self.cube_callback)

    def load_distortion_parameters(self) -> None:
        '''
        Load camera calibration coefficients from file
        '''
        # Get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        self.mat_data = sio.loadmat(os.path.join(rospack.get_path('hyper_drive'),'config','distortion',f'{self.model}_params.mat'))
        self.camera_matrix = self.mat_data['intrinsicMatrix']
        self.dist_coeffs = self.mat_data['distortionCoefficients']
        
        
    def cube_callback(self, msg: DataCube) -> None:
        '''
        Callback function for hypercube data
        '''
        cube = np.reshape(msg.data, (msg.width, msg.height, msg.lam))
        if self.new_camera_matrix.size == 0:
            self.new_camera_matrix, _ = cv.getOptimalNewCameraMatrix(
                self.camera_matrix, self.dist_coeffs, (msg.width, msg.height), 1, (msg.width, msg.height)
            )
        # Create empty cubes with the same shape
        corrected_cube = np.zeros_like(cube, dtype=np.float32)
        # Apply corrective function along the channels
        for i in range(msg.lam):
            corrected_cube[:,:,i] = cv.undistort(cube[:,:,i], self.camera_matrix,self.dist_coeffs,None,self.new_camera_matrix)
        # Send out the new datacube
        toSend = DataCube()
        # Create header
        h = Header()
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        toSend.header = h
        toSend.data = corrected_cube.flatten()
        toSend.width,toSend.height,toSend.lam = msg.width, msg.height, msg.lam
        toSend.central_wavelengths,toSend.qe,toSend.fwhm_nm = msg.central_wavelengths, msg.qe, msg.fwhm_nm
        self.pub.publish(toSend)


    def shutdown(self):
        '''
        Custom shutdown behavior
        '''
        return

if __name__ == '__main__':
    rospy.init_node('CubeCorrecter', anonymous=True)
    try:
        my_node = CubeCorrecter()
        rospy.spin()
    except rospy.ROSInterruptException:
        my_node.shutdown()
