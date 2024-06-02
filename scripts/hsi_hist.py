#!/usr/bin/env python3

import rospy
import io
from PIL import Image
import cv2 as cv
import numpy as np
import ros_numpy
import matplotlib.pyplot as plt
import seaborn as sns
from hyper_drive.msg import MultipleDataCubes
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from sensor_msgs.msg import *
from std_msgs.msg import *
# from cv_bridge import CvBridge

class HSI_HIST():
    def __init__(self):
        plt.switch_backend('agg')
        # Current wavelength to display
        self.lam = 0
        # Flag to set if new information is available
        self.new_cube = False
        # Placeholder for hypercube data
        self.cube = np.array([])
        # Placeholder ofr single channel grayscale image
        self.img = np.array([])
        
        self.cam_model = rospy.get_param('~camera_model', default='ximea')

        #get cam model
        self.model_sub = rospy.Subscriber('/hsi_gui/camera', String, self.callback_model)

        #get slider num
        self.slider_num = rospy.Subscriber('/hsi_gui/channel', Int8, self.callback_slider)

        #get cube data
        # TODO - make this a rosparam for the topic
        self.cubes_sub = rospy.Subscriber('/synchronous_cubes', MultipleDataCubes, self.callback_cubes)

        # self.imec_data_sub = rospy.Subscriber('/imec/undistort_data', DataCube, self.imec_callback)
        # self.ximea_data_sub = rospy.Subscriber('/ximea/undistort_data', DataCube, self.ximea_callback)
        # self.combined_data_sub = rospy.Subscriber('/combined/undistort_data', DataCube, self.combined_callback)

        #publish image
        self.pub_img = rospy.Publisher('/hsi_gui/channel_img', Image, queue_size=10)

        #publish histogram
        self.pub_hist = rospy.Publisher('/hsi_gui/hist_img', Image, queue_size=10)
        
    def callback_model(self, msg):
        '''
        Change model and allow only a single topic to run things
        '''
        self.cam_model = msg.data

    def callback_slider(self, msg):
        self.lam = msg.data

    def callback_cubes(self, msg):
        self.ximea_callback(msg.cubes[0])
        self.imec_callback(msg.cubes[1])
        self.vimba_callback(msg.im)

    def imec_callback(self, msg):
        if self.cam_model == 'imec':
            # Mark that we've received a new cube
            self.new_cube = True
            self.cube = np.reshape(msg.data, (msg.width, msg.height, msg.lam))
            self.do_update()

    def ximea_callback(self, msg):
        if self.cam_model == 'ximea':
            # Mark that we've received a new cube
            self.new_cube = True
            self.cube = np.reshape(msg.data, (msg.width, msg.height, msg.lam))
            self.do_update()

    def vimba_callback(self, msg):
        if self.cam_model == 'vimba':
            # Mark that we've received a new cube
            self.new_cube = True
            self.cube = ros_numpy.numpify(msg)

            # cv.imshow('vimba', self.cube[:,:,0])
            # cv.waitKey(1)

            #np.reshape(msg.data, (msg.width, msg.height, msg.lam))
            self.do_update()

    def do_update(self):
        '''
        Callback function for hypercube data
        '''

        # Every time we receive a new cube, we should republish the data to both channels
        self.update_histogram()
        self.update_image()

    def rescale_image(self, arr):
        '''
        Rescale image to 8-bit pixels for display with PyQT
        '''
        if self.cam_model == 'imec':
            # resize image
            scale_percent = 175 # percent of original size
            width = int(arr.shape[1] * scale_percent / 100)
            height = int(arr.shape[0] * scale_percent / 100)
            dim = (width, height)
            arr = cv.resize(arr,  dim, interpolation = cv.INTER_AREA)
        if self.cam_model == 'vimba':
             # resize image
            scale_percent = 25 # percent of original size
            width = int(arr.shape[1] * scale_percent / 100)
            height = int(arr.shape[0] * scale_percent / 100)
            dim = (width, height)
            arr = cv.resize(arr,  dim, interpolation = cv.INTER_AREA)
        return ((arr) * (1/((arr.max())) * 255)).astype('uint8')

    def update_histogram(self):
        '''
        Redraw the histogram on the main panel
        '''
        # configure and draw the histogram figure
        if self.img.size > 0:
            histogram, bin_edges = np.histogram(self.cube[:, :, self.lam], bins=50)
            fig = plt.figure(frameon=False)
            canvas = FigureCanvas(fig)
            ax = fig.gca()
            sns.histplot(
                self.cube[:,:,self.lam].flatten(),
                bins=100
            )
            ax.set_title("Grayscale Histogram")
            ax.set_xlabel("grayscale value")
            ax.set_ylabel("pixel count")
            canvas.draw()
            buf = io.BytesIO()
            plt.savefig(buf, format='png', transparent=True)
            buf.seek(0)
            file_bytes = np.asarray(bytearray(buf.read()), dtype=np.uint8)
            img = cv.imdecode(file_bytes, cv.IMREAD_UNCHANGED)
            ros_image = ros_numpy.msgify(Image, img, encoding="8UC4")
            self.pub_hist.publish(ros_image)
            plt.close()

    def update_image(self, override = False):
        '''
        Update the value of the single channel image
        '''
        if self.cube.size == 0:
            return
        if self.new_cube == False and override == False:
            return
        slice_image = self.cube[:, :, self.lam]
        self.img = slice_image
        img = self.rescale_image(slice_image)
        self.img = img   
        backtorgb = cv.cvtColor(self.img,cv.COLOR_GRAY2RGB)
        ros_image = ros_numpy.msgify(Image, backtorgb, encoding="8UC3")
        self.pub_img.publish(ros_image)
        self.new_cube = False
        

if __name__ == "__main__":
    rospy.init_node('HSIProcessor', anonymous=True)
    try:
        my_node = HSI_HIST()
        rospy.spin()
    except rospy.ROSInterruptException:
        my_node.shutdown()