#!/usr/bin/env python3

from hyper_drive.msg import MultipleDataCubes
from hyper_drive.msg import DataCube
import os
import sys
import cv2
import rospy
import typing
import rospkg
import threading
import traceback
import ros_numpy
import numpy as np
from bs4 import BeautifulSoup
from numba import jit, prange
from pathlib import Path
import matplotlib.pyplot as plt
from hyper_drive.srv import adjust_param
from std_msgs.msg import Header
import logging
from sensor_msgs.msg import Image

### Set parameters to find the HSI mosaic binaries
os.environ['PATH'] += os.pathsep + r'/opt/imec/hsi-mosaic/bin'
# Also add them to the path
sys.path.append('/opt/imec/hsi-mosaic/python_apis')
# Import IMEC specific libraries
import hsi_common as HSI_COMMON
import hsi_camera as HSI_CAMERA
import hsi_mosaic as HSI_MOSAIC

class DataCubesGenerator(object):
    def __init__(self):
        # Setup logging to roslog location
        log_dir = rospkg.get_log_dir()
        rospy.loginfo("Logging camera info to {}".format(log_dir))
        HSI_COMMON.InitializeLogger(log_dir, HSI_COMMON.LoggerVerbosity.LV_INFO)
        rospy.loginfo("Loading Context ...")
        # Lock for camera setting updates
        self.lock = threading.Lock()
        # Setup callback for data
        self.ros_pack = rospkg.RosPack()
        #sleep time amoutn param
        self.sleep_time = rospy.get_param('~sleep', 0.001)
        #ximea
        self.x_frame_rate = rospy.get_param('~x_frame_rate', 30)
        self.x_integration_time = rospy.get_param('~x_integration_time', 15)
        #imec
        self.i_frame_rate = rospy.get_param('~i_frame_rate', 10)
        self.i_integration_time = rospy.get_param('~i_integration_time', 70)
        #frequency of publishing
        self.time_wait = rospy.get_param('~time_wait', 0) * 60
        self.param_server = rospy.Service('adjust_param', adjust_param, self.handle_adjust_param)
        # Create publisher to send datacubes on
        self.pub_cube = rospy.Publisher('synchronous_cubes', MultipleDataCubes, queue_size=10)
        #subscribe to Vimba raw image
        self.sub_img = rospy.Subscriber(f'/camera/image_raw', Image, self.image_callback)
        self.raw_img = np.zeros((100,100,3),dtype=np.uint8)
        # Load camera parameters
        self.parse_parameters('ximea')
        self.parse_parameters('imec')
        # Initialize the ximea and imec cameras
        self.initialize_camera('ximea')
        self.initialize_camera('imec')
        # Ximea Explicit Allocate based on Output-Data-Format
        x_dataformat = HSI_CAMERA.GetOutputFrameDataFormat (self.x_device)
        self.x_frame = HSI_COMMON.AllocateFrame(x_dataformat)
        # Imec Explicit Allocate based on Output-Data-Format
        i_dataformat = HSI_CAMERA.GetOutputFrameDataFormat(self.i_device)
        self.i_frame = HSI_COMMON.AllocateFrame(i_dataformat)
        self.setup_context('ximea')
        self.setup_context('imec')
        self.time_future = rospy.Time.now().to_sec() + self.time_wait

    #callback to handle vimba raw image
    def image_callback(self, msg: Image) -> None:
        self.raw_img = ros_numpy.numpify(msg)

    def initialize_camera(self, model) -> None:
        # Rate at which to generate composite data cubes
        # Look for connected cameras an choose appropriate model (assumes we only have 1 IMEC and 1 XIMEA)
        if model == 'ximea':
            integration_range = (0.021000, 999.995000)
            dev_list = HSI_CAMERA.EnumerateConnectedDevices(manufacturer=HSI_CAMERA.Manufacturer.EM_XIMEA)
            roi = [HSI_COMMON.RegionOfInterest(x=0, y=0, width=2045, height=1085)]

            # Connect to the first available camera of the specified model type
            rospy.loginfo('looking for device:: {}'.format(dev_list[0]))
            self.x_device = HSI_CAMERA.OpenDevice(dev_list[0])
            HSI_CAMERA.SetRegionOfInterestArray(self.x_device, roi)
            rospy.loginfo("Region-of-Intereset Set to: {}".format(roi))
            rospy.loginfo('Initializing Camera...')
            HSI_CAMERA.Initialize(self.x_device)
            # Get/Set Camera Configuration Parameters (example)
            c_params = HSI_CAMERA.GetConfigurationParameters(self.x_device)
            rospy.loginfo("C PARAMS>")
            rospy.loginfo(c_params)
            r_params = HSI_CAMERA.GetRuntimeParameters(self.x_device)
            rospy.loginfo(r_params)
            # Set the frame rate and exposure time before the camera starts
            # These values can be adjusted later through a callback
            
            r_params.frame_rate_hz = self.x_frame_rate
            r_params.exposure_time_ms = self.x_integration_time

            # Set these parameters on the device
            HSI_CAMERA.SetRuntimeParameters(self.x_device, r_params)

            rospy.loginfo("R PARAMS>")
            rospy.loginfo(r_params)
        
        elif model == 'imec':
            integration_range = (0.010000, 90)
            dev_list = HSI_CAMERA.EnumerateConnectedDevices(manufacturer=HSI_CAMERA.Manufacturer.EM_IMEC)
            roi = [HSI_COMMON.RegionOfInterest(x=1, y=1, width=639, height=510)]

            # Connect to the first available camera of the specified model type
            rospy.loginfo('looking for device:: {}'.format(dev_list[0]))
            self.i_device = HSI_CAMERA.OpenDevice(dev_list[0])
            HSI_CAMERA.SetRegionOfInterestArray(self.i_device, roi)
            rospy.loginfo("Region-of-Intereset Set to: {}".format(roi))
            rospy.loginfo('Initializing Camera...')
            HSI_CAMERA.Initialize(self.i_device)
            # Get/Set Camera Configuration Parameters (example)
            c_params = HSI_CAMERA.GetConfigurationParameters(self.i_device)
            rospy.loginfo("C PARAMS>")
            rospy.loginfo(c_params)
            r_params = HSI_CAMERA.GetRuntimeParameters(self.i_device)
            rospy.loginfo(r_params)
            # Set the frame rate and exposure time before the camera starts
            # These values can be adjusted later through a callback

            r_params.frame_rate_hz = self.i_frame_rate
            r_params.exposure_time_ms = self.i_integration_time
        
            r_params['flip_vertical'] = False
            r_params['flip_horizontal'] = True
       
            # Set these parameters on the device
            HSI_CAMERA.SetRuntimeParameters(self.i_device, r_params)
            
            rospy.loginfo("R PARAMS>")
            rospy.loginfo(r_params)

    def restart_camera(self) -> None:
        '''
        Restart camera in the event there is an error setting the runtime parameters
        '''
        rospy.logerr('ERROR IN CAMERA MAIN LOOP! Waiting 5 seconds and reinitializing the camera')
        HSI_CAMERA.Pause(self.x_device)
        HSI_CAMERA.Stop(self.x_device)
        HSI_CAMERA.CloseDevice(self.x_device)

        HSI_CAMERA.Pause(self.i_device)
        HSI_CAMERA.Stop(self.i_device)
        HSI_CAMERA.CloseDevice(self.i_device)
        # Sleep 5 seconds
        rospy.sleep(5)
        # Restart the camera
        self.initialize_camera('ximea')
        self.initialize_camera('imec')


    def setup_context(self, model) -> None:
        '''
        Load HSI Context files and prepare to run demosaicing pipeline
        '''
        dn_context = os.path.join(self.ros_pack.get_path('hyper_drive'),'config',model,'context')
        #####################################################################
        version = HSI_MOSAIC.GetAPIVersion()
        rospy.loginfo(f'VERSION :: {version}')
        rospy.loginfo(Path(dn_context).absolute())
        assert Path(dn_context).exists()
        self.context = HSI_MOSAIC.LoadContext(dn_context)
        rospy.loginfo(f'Context = {self.context}')
        status = HSI_MOSAIC.ContextGetStatus(self.context)
        rospy.loginfo(status)
        
        if model == 'ximea':
            self.x_pipeline = HSI_MOSAIC.Create(self.context)
            rospy.loginfo(self.x_pipeline)

            self.params = HSI_MOSAIC.GetConfigurationParameters(self.x_pipeline)
            self.params.spatial_median_filter_enable = False
            rospy.loginfo(f'Configuration Params : {self.params}')
            HSI_MOSAIC.SetConfigurationParameters(self.x_pipeline, self.params)
            rospy.loginfo('Initializing Pipeline')
            HSI_MOSAIC.Initialize(self.x_pipeline)

            outputdataformat = HSI_MOSAIC.GetOutputDataFormat(self.x_pipeline)
            rospy.loginfo(f'Output Data Format = {outputdataformat}')

            self.x_cube = HSI_COMMON.AllocateCube(outputdataformat)

            rospy.loginfo('Starting Pipeline')
            HSI_MOSAIC.Start(self.x_pipeline)

        elif model == 'imec':
            self.i_pipeline = HSI_MOSAIC.Create(self.context)
            rospy.loginfo(self.i_pipeline)

            self.params = HSI_MOSAIC.GetConfigurationParameters(self.i_pipeline)
            self.params.spatial_median_filter_enable = False
            rospy.loginfo(f'Configuration Params : {self.params}')
            HSI_MOSAIC.SetConfigurationParameters(self.i_pipeline, self.params)
            rospy.loginfo('Initializing Pipeline')
            HSI_MOSAIC.Initialize(self.i_pipeline)

            outputdataformat = HSI_MOSAIC.GetOutputDataFormat(self.i_pipeline)
            rospy.loginfo(f'Output Data Format = {outputdataformat}')

            self.i_cube = HSI_COMMON.AllocateCube(outputdataformat)

            rospy.loginfo('Starting Pipeline')
            HSI_MOSAIC.Start(self.i_pipeline)

    def parse_parameters(self, model) -> None:
        '''
        Load parameter for camera from manufacturer provided XML file
        for publication in datacube messages
        '''
        # Get an instance of RosPack with the default search paths
        param_path = os.path.join(self.ros_pack.get_path('hyper_drive'),'config',f'{model}.xml')
        with open(param_path, 'r') as f:
            data = f.read()
            Bs_data = BeautifulSoup(data, "xml")
            self.x_central_wave = []
            self.i_central_wave = []
            self.x_fwhm = []
            self.i_fwhm = []
            self.x_QE = []
            self.i_QE = []
            for band in Bs_data.find_all("wavelength_nm"):
                if model == 'ximea':
                    self.x_central_wave.append(float(band.getText()))
                elif model == 'imec':
                    self.i_central_wave.append(float(band.getText()))
            for band in Bs_data.find_all("fwhm_nm"):
                if model == 'ximea':
                    self.x_fwhm.append(float(band.getText()))
                elif model == 'imec':
                    self.i_fwhm.append(float(band.getText()))
            for band in Bs_data.find_all("QE"):
                if model == 'ximea':
                    self.x_QE.append(float(band.getText()))
                elif model == 'imec':
                    self.i_QE.append(float(band.getText()))
            # Get calibration coefficients
            coefficients = []
            for coefficient in Bs_data.find_all("coefficients"):
                coefficients.append(np.array([float(z) for z in coefficient['values'].split()]))
            self.coefficients = np.array(coefficients)
        
    def handle_adjust_param(self, req: adjust_param) -> adjust_param:
        '''    Listen to user parameter requests    '''     
        frame_time = 1/(req.frame_rate) * 1000    
        rospy.loginfo(f'Incoming message: {adjust_param}')
        # Check the time bounds are OK
        if req.camera_model == 'imec':
            cam = self.i_device
            integration_range = (0.010000, 90)
        if req.camera_model == 'ximea':
            cam = self.x_device
            integration_range = (0.021000, 999.995000)

        if (integration_range[0] < req.integration_time < integration_range[1]) & (req.integration_time < frame_time):
            return self.set_camera_params(cam, req.integration_time, req.frame_rate)
        else:
            rospy.logerr('INVALID INTEGRATION TIME REQUESTED!')
            return False

    def set_camera_params(self, camera, exposure_time: float, frame_rate: float) -> bool:
        try:
            # Pause the camera        
            print(f'Exposure time: {exposure_time} Frame Rate: {frame_rate}')
            # Proposing to try 3 steps
            with self.lock:         
                #Step-1: Set frame rate to a very low value eg. 1 fps            
                HSI_CAMERA.Pause(camera)
                # Get r-params
                r_params = HSI_CAMERA.GetRuntimeParameters(camera)
                print(r_params)
                r_params.frame_rate_hz = 1            
                rospy.loginfo(f'Update Runtime Params: {HSI_CAMERA.SetRuntimeParameters(camera, r_params)}')       
                # #Step-2: Set integration time            
                r_params = HSI_CAMERA.GetRuntimeParameters(camera)
                r_params.exposure_time_ms = exposure_time            
                rospy.loginfo(f'Update Runtime Params: {HSI_CAMERA.SetRuntimeParameters(camera, r_params)}')       
                # #Step-3: Set frame rate            
                r_params = HSI_CAMERA.GetRuntimeParameters(camera)
                r_params.frame_rate_hz = frame_rate           
                rospy.loginfo(f'Update Runtime Params: {HSI_CAMERA.SetRuntimeParameters(camera, r_params)}')       
                HSI_CAMERA.Start(camera)
                rospy.sleep(0.1)
                return True        

        except Exception as e:
            rospy.logerr(traceback.print_exc())
            rospy.logerr(str(e))
            rospy.logerr('Error setting user user parameter!')
            return False



    def undistort(self, cube, model):
        #criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1)

        imec_K = [[571.0648167, 0, 96.21785],
                  [0, 572.4372833, 77.75815],
                  [0, 0, 1]]
        imec_distort = [-0.715016666666667, 18.2009666666667, -0.003883333333333, -0.002716666666667, -234.22125]

        ximea_K = [[929.0870667, 0, 193.47025],
                   [0, 928.9873333, 114.0106],
                   [0, 0, 1]]
        ximea_distort = [0.12115, -5.45603333333333, 0.0021, -0.004283333333333, 100.424166666667]
    
        if model == 'imec':
            for lam in range(cube.shape[2]):
                undistorted_img = cv2.undistort(cube[:, :, lam], np.matrix(imec_K), np.array(imec_distort))
                cube[:, :, lam] = undistorted_img

        elif model == 'ximea':
            for lam in range(cube.shape[2]):
                undistorted_img = cv2.undistort(cube[:, :, lam], np.matrix(ximea_K), np.array(ximea_distort))
                cube[:, :, lam] = undistorted_img

    def publish_cubes(self, x_cube: np.ndarray, i_cube: np.ndarray):
        '''
        Publish hyperspectral datacubes
        '''

        # self.undistort(x_cube, 'ximea')
        # self.undistort(i_cube, 'imec')

        # print(f'XIMEA: {x_cube.shape}')
        # print(f'IMEC: {i_cube.shape}')
        # print(f'VIMBA: {self.raw_img.shape}')
        tcube = ((x_cube[:,:,0]) * (1/((x_cube[:,:,0].max())) * 255)).astype('uint8')
        tcube = ((i_cube[:,:,0]) * (1/((i_cube[:,:,0].max())) * 255)).astype('uint8')
        #Messages
        x_ros_cube = DataCube()
        i_ros_cube = DataCube()
        ros_cubes = MultipleDataCubes()
        np.save('/home/river/x_cube.npy', x_cube)
        np.save('/home/river/i_cube.npy', i_cube)
        # Create header
        h = Header()
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        
        x_ros_cube.header = h
        x_ros_cube.data = x_cube.flatten() 
        x_ros_cube.width, x_ros_cube.height, x_ros_cube.lam = tuple(x_cube.shape)
        x_ros_cube.qe = self.x_QE
        x_ros_cube.fwhm_nm = self.x_fwhm
        x_ros_cube.central_wavelengths = self.x_central_wave
        
        i_ros_cube.header = h
        i_ros_cube.data = i_cube.flatten() 
        i_ros_cube.width, i_ros_cube.height, i_ros_cube.lam = tuple(i_cube.shape)
        i_ros_cube.qe = self.i_QE
        i_ros_cube.fwhm_nm = self.i_fwhm
        i_ros_cube.central_wavelengths = self.i_central_wave
        ros_cubes.cubes = [x_ros_cube, i_ros_cube]

        ros_cubes.im = ros_numpy.msgify(Image, self.raw_img, encoding="8UC3")
        self.pub_cube.publish(ros_cubes)


    def run(self) -> None:
        '''
        Run central processing loop for camera
        '''
        HSI_CAMERA.Start(self.x_device)
        HSI_CAMERA.Start(self.i_device)
        while not rospy.is_shutdown():
            if rospy.Time.now().to_sec() > self.time_future:
                self.time_future = rospy.Time.now().to_sec() + self.time_wait
                try:
                    with self.lock:
                    # Send a Software Trigger to the camera and grab the Frame
                        HSI_CAMERA.Trigger(self.x_device)
                        HSI_CAMERA.Trigger(self.i_device)

                        HSI_CAMERA.AcquireFrame(self.x_device, frame=self.x_frame)
                        HSI_CAMERA.AcquireFrame(self.i_device, frame=self.i_frame)               

                        HSI_MOSAIC.PushFrame(self.x_pipeline, self.x_frame)
                        HSI_MOSAIC.PushFrame(self.i_pipeline, self.i_frame)

                        HSI_MOSAIC.GetCube(self.x_pipeline, self.x_cube, timeout_ms=1000)
                        HSI_MOSAIC.GetCube(self.i_pipeline, self.i_cube, timeout_ms=1000)
                        
                        x_py_cube = HSI_COMMON.CubeAsArray(self.x_cube, BSQ=False)
                        i_py_cube = HSI_COMMON.CubeAsArray(self.i_cube, BSQ=False)
                        rospy.loginfo(f'Cube shapes XIMEA: {x_py_cube.shape}, IMEC: {i_py_cube.shape}, VIMBA: {self.raw_img.shape}')
                        self.publish_cubes(x_py_cube, i_py_cube)

                except Exception as e:
                    rospy.logerr(e)
                    rospy.logerr(traceback.print_exc())
                    rospy.logerr('Exception in main capture loop!')
                    self.restart_camera()
                rospy.sleep(self.sleep_time)
        # End of loop behavior        
        self.shutdown()

    def shutdown(self):
        '''
        Custom shutdown behavior
        '''
        rospy.loginfo("Cleaning up node for the cameras")
        HSI_CAMERA.Pause(self.x_device)
        HSI_CAMERA.Stop(self.x_device)
        HSI_CAMERA.CloseDevice(self.x_device)
        HSI_MOSAIC.Pause(self.x_pipeline)
        HSI_MOSAIC.Stop(self.x_pipeline)
        HSI_COMMON.DeallocateCube(self.x_cube)
        HSI_COMMON.DeallocateFrame(self.x_frame)
        HSI_MOSAIC.DeallocateContext(self.context)

        HSI_CAMERA.Pause(self.i_device)
        HSI_CAMERA.Stop(self.i_device)
        HSI_CAMERA.CloseDevice(self.i_device)
        HSI_MOSAIC.Pause(self.i_pipeline)
        HSI_MOSAIC.Stop(self.i_pipeline)
        HSI_COMMON.DeallocateCube(self.i_cube)
        HSI_COMMON.DeallocateFrame(self.i_frame)

if __name__ == '__main__':
    rospy.init_node('DataCubesProcessor', anonymous=True)
    try:
        my_node = DataCubesGenerator()
        my_node.run()
    except rospy.ROSInterruptException:
        my_node.shutdown()
