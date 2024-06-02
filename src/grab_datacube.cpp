#include <ros/ros.h>
#include <hsi_camera_api.h>
// #include <hsi_camera_api.h>

#include <string>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include "hyper_drive/DataCube.h"

#include <cv_bridge/cv_bridge.h>
#include "hyper_drive/adjust_param.h"

// Modified main.cpp from hsi-mosiac/hsi_camera_api/example_C_api
// Authored by Gary Lvov and Nathaniel Hanson

class DatacubeGrabber
{
public:
    DatacubeGrabber(ros::NodeHandle *nh)
    {
        ros::ServiceServer service = nh->advertiseService("set_integration_time", &DatacubeGrabber::adjust_cam_param, this);

        this->raw_pub = nh->advertise<sensor_msgs::Image>("spectral_data", 10);
        ros::param::get("~camera_model", this->model);
        if (this->model.compare("imec") == 0)
        {
            this->integration_range = std::make_pair(0.010000, 90);
        }
        else if (this->model.compare("ximea") == 0)
        {
            this->integration_range = std::make_pair(0.021000, 999.995000);
        }

        this->ret_val = HSI_OK;
        this->ret_val = commonInitializeLogger("./logs/", LV_DEBUG);
        this->DisplayResult("commonInitializeLogger", this->ret_val);

        CameraInfo camera_info = this->get_device_info(0);
        this->camera = 0;

        this->ret_val = cameraOpenDevice(&this->camera, camera_info);
        this->DisplayResult("cameraOpenDevice", this->ret_val);

        this->config_parameters = {};
        cameraGetConfigurationParameters(this->camera, &this->config_parameters);
        // change configuration parameters here
        // note: also runtime parameters can be changed at this point
        this->ret_val = cameraSetConfigurationParameters(this->camera, this->config_parameters);
        this->DisplayResult("cameraSetConfigurationParameters", ret_val);

        this->ret_val = cameraInitialize(this->camera);
        this->DisplayResult("cameraInitialize", this->ret_val);

        this->data_format = {};
        this->ret_val = cameraGetOutputFrameDataFormat(this->camera, &this->data_format);
        this->DisplayResult("cameraGetOutputFrameDataFormat", this->ret_val);

        this->frame = {};
        this->ret_val = commonAllocateFrame(&this->frame, this->data_format);
        this->DisplayResult("commonAllocateFrame", this->ret_val);

        this->runtime_parameters = {};
        cameraGetRuntimeParameters(this->camera, &this->runtime_parameters);
        this->runtime_parameters.frame_rate_hz = 100;
        this->runtime_parameters.exposure_time_ms = 10;
        this->runtime_parameters.trigger_mode = TM_NoTriggering;
        this->ret_val = cameraSetRuntimeParameters(this->camera, this->runtime_parameters);
        this->DisplayResult("cameraSetRuntimeParameters", this->ret_val);

        this->stream_camera_feed();
    }

    ~DatacubeGrabber()
    {
        cameraPause(camera);
        cameraStop(camera);
        commonDeallocateFrame(&frame);
        commonDeallocateFrameDataFormat(&data_format);
        cameraCloseDevice(&this->camera);
        system("pause");
    }

private:
    ros::Publisher raw_pub;
    ros::ServiceServer adjust_cam_param_srv;

    std::string model;
    std::pair<double, double> integration_range;
    HSI_RETURN ret_val;
    HANDLE camera;
    cameraConfigurationParameters config_parameters;
    cameraRuntimeParameters runtime_parameters;
    FrameDataFormat data_format;
    FrameFloat frame;

    std::vector<float> flattened_data;

    void stream_camera_feed()
    {
        this->ret_val = cameraStart(this->camera);
        
        int width = this->frame.format.width;
        int height = this->frame.format.height;
        cv::Mat image = cv::Mat::zeros(width, height, CV_32F);
        cv_bridge::CvImage cv_image;
        cv_image.encoding = "32FC1";    
        sensor_msgs::Image msg;
        while (ros::ok()) {
            
            cv_image.header.stamp = ros::Time::now();
            this->ret_val =cameraTrigger(this->camera);
            this->ret_val = cameraAcquireFrame(this->camera, &this->frame);
            for(int row = 0; row < height; row++) {
                for(int col = 0; col < width; col++) {
                    image.at<float>(row, col) = this->frame.pp_data[row][col];
                }
            }
            cv_image.image = image;
            cv_image.toImageMsg(msg);
            this->raw_pub.publish(msg);
            ros::spinOnce();
        }
    }

    bool adjust_cam_param(hyper_drive::adjust_param::Request &req,
                          hyper_drive::adjust_param::Response &res)
    {
        res.success = true;
        if (req.integration_time > this->integration_range.first && req.integration_time < this->integration_range.second)
        {
            ROS_INFO("%f, %f",this->integration_range.first, this->integration_range.second);
            ROS_INFO("%f",req.integration_time);
            cameraPause(this->camera);
            cameraGetRuntimeParameters(this->camera, &this->runtime_parameters);
            this->runtime_parameters.exposure_time_ms = req.integration_time;
            cameraSetRuntimeParameters(this->camera, this->runtime_parameters);
            this->ret_val = cameraStart(this->camera);
            return true;
        }
        else
        {
            res.success = false;
        }
        return true;
    }

    CameraInfo get_device_info(int selected_camera)
    {
        // get the HSI Common API version
        int major = 0, minor = 0, patch = 0, build = 0;
        this->ret_val = commonGetAPIVersion(&major, &minor, &patch, &build);
        this->DisplayResult("commonGetAPIVersion", this->ret_val);
        // if (HSI_OK != this->ret_val)
        // {
        //     system("pause");
        //     return -1;
        // }

        // print the version number
        std::cout << "*** HSI Common API version: " << major << "." << minor << "." << patch << "." << build << std::endl;

        // get the HSI Camera API version
        this->ret_val = cameraGetAPIVersion(&major, &minor, &patch, &build);
        this->DisplayResult("cameraGetAPIVersion", this->ret_val);
        // if (HSI_OK != this->ret_val)
        // {
        //     system("pause");
        //     return -1;
        // }

        // print the version number
        std::cout << "*** HSI Camera API version: " << major << "." << minor << "." << patch << "." << build << std::endl;

        // allocate space for 5 CameraInfo's
        int const nr_preallocated_devices = 5;
        CameraInfo camera_infos[nr_preallocated_devices];

        // enumerate the connected devices
        int nr_devices = 0;
        if (this->model.compare("imec") == 0) {
            this->ret_val = cameraEnumerateConnectedDevices(&camera_infos[0], &nr_devices, nr_preallocated_devices, {EM_IMEC});
        } else if (this->model.compare("ximea") == 0) {
            this->ret_val = cameraEnumerateConnectedDevices(&camera_infos[0], &nr_devices, nr_preallocated_devices, {EM_XIMEA});
        }
        this->DisplayResult("cameraEnumerateConnectedDevices", this->ret_val);
        // if (HSI_OK != this->ret_val)
        // {
        //     system("pause");
        //     return -1;
        // }

        // if (0 == nr_devices)
        // {
        //     std::cout << "No connected devices found." << std::endl;
        //     system("pause");
        //     return 0;
        // }

        std::cout << "Number of connected devices: " << nr_devices << std::endl;

        for (int i = 0; i < nr_devices; ++i)
        {
            CameraInfo *p_camera_info = &camera_infos[i];

            std::cout << "Camera " << i << std::endl;
            std::cout << "    manufacturer:  " << p_camera_info->manufacturer << std::endl;
            std::cout << "    model:  " << p_camera_info->model << std::endl;
        }

        std::cout << std::endl;
        CameraInfo camera_info = camera_infos[selected_camera];

        std::cout << "Using camera " << selected_camera << std::endl;
        std::cout << "    manufacturer:  " << camera_info.manufacturer << std::endl;
        std::cout << "    model:  " << camera_info.model << std::endl;
        std::cout << "    serial_number:  " << camera_info.serial_number << std::endl;
        std::cout << "    identification_string:  " << camera_info.identification_string << std::endl;
        std::cout << std::endl;

        return camera_info;
    }

    void DisplayResult(std::string const &i_message, HSI_RETURN i_result)
    {
        std::cout << "*** " << std::setw(35) << i_message << ": " << i_result << " => ";

        switch (i_result)
        {
        case HSI_OK:
            std::cout << "Function call successful.\n";
            break;
        case HSI_HANDLE_INVALID:
            std::cout << "Invalid device handle specified.\n";
            break;
        case HSI_ARGUMENT_INVALID:
            std::cout << "Invalid argument provided in function call.\n";
            break;
        case HSI_CALL_ILLEGAL:
            std::cout << "Function call illegal given the current internal state.\n";
            break;
        case HSI_FILE_NOT_FOUND:
            std::cout << "A file could not be found.\n";
            break;
        case HSI_CALIBRATION_FILE_NOT_FOUND:
            std::cout << "Sensor calibration file could not be found.\n";
            break;
        case HSI_CONNECTION_FAILED:
            std::cout << "Camera could not be connected to the system.\n";
            break;
        case HSI_ALLOCATION_ERROR:
            std::cout << "Allocation of resources failed.\n";
            break;
        case HSI_ACQUISITION_TIMEOUT:
            std::cout << "An Acquisition timeout has occured.\n";
            break;
        case HSI_ACQUISITION_FAILED:
            std::cout << "Acquisition failed during operation.\n";
            break;
        case HSI_DATA_NOT_ALLOCATED:
            std::cout << "Provided data structure is not allocated.\n";
            break;
        case HSI_DATA_NOT_VALID:
            std::cout << "Data with valid flag false provided as input for operation.\n";
            break;
        case HSI_DATA_NOT_COMPATIBLE:
            std::cout << "Data provided is not compatible.\n";
            break;
        case HSI_FILE_SYSTEM_ERROR:
            std::cout << "Specified directory doesn't exist and could not be created.\n";
            break;
        case HSI_FILE_IO_ERROR:
            std::cout << "Could not read or write data from the filesystem.\n";
            break;
        case HSI_INTERNAL_ERROR:
            std::cout << "An unexpected internal error occurred.\n";
            break;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hyper_drive", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    DatacubeGrabber nc = DatacubeGrabber(&nh);
}