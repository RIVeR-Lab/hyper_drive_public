#include <ros/ros.h>
// #include <ros/package.h>
#include <hsi_camera_api.h>
#include <hsi_mosaic_api.h>
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
// Authored by Nathaniel Hanson

class ImageDemosaicer
{
public:
    ImageDemosaicer(ros::NodeHandle *nh)
    {
        // Load the package path
        this->package_path = "/home/river/catkin_ws/src/hyper_drive"; // ros::package::getPath("hyper_drive");
        ros::param::get("~camera_model", this->model);
        this->raw_sub = nh->subscribe("cube_grabber/spectral_data", 10, &ImageDemosaicer::cube_call_back, this);
        this->cube_pub = nh->advertise<hyper_drive::DataCube>("corrected_cube", 1);

        if (this->model.compare("imec") == 0)
        {
            // Set acquisition context for IMEC
            this->context_path = this->package_path + "/config/imec/context";
            // this->blank_cube_path = this->package_path + "/config/empty_cubes/blank_ximea.raw.xml";
            this->output_width = 640;
            this->output_height = 510;
            this->DisplayResult("LoadFrame()", commonLoadFrame(&data_frame, this->blank_cube_path.c_str()));
        }
        else if (this->model.compare("ximea") == 0)
        {
            // Set acquisition context for XIMEA
            this->context_path = this->package_path + "/config/ximea/context";
            // this->blank_cube_path = this->package_path + "/config/empty_cubes/blank_imec.raw.xml";
            this->output_width = 2045;
            this->output_height = 1085;
            //this->DisplayResult("LoadFrame()", commonLoadFrame(&data_frame, this->blank_cube_path.c_str()));
        }

        this->ret_val = HSI_OK;
        this->ret_val = commonInitializeLogger("./logs/", LV_VERBOSE);
        this->DisplayResult("commonInitializeLogger", this->ret_val);
        this->DisplayResult("LoadContext", mosaicLoadContext(&context, this->context_path.c_str()));
        // set some pipeline configuration parameters
        mosaicConfigurationParameters parameters = {};
        parameters.spatial_median_filter_enable = true;
        parameters.spatial_median_filter_kernel_size = KS_3X3;
        parameters.spatial_resampling_width = this->output_width;
        parameters.spatial_resampling_height = this->output_height;
        parameters.white_balance_enable = false;

        // we need some variables along the way
        this->pipeline = 0x0;
        this->output_data_format = {};
        this->data_cube = {};
        // build and configure pipeline
        this->DisplayResult("Create()", mosaicCreate (&pipeline, context));
        this->DisplayResult("SetConfigurationParameters()", mosaicSetConfigurationParameters (pipeline, parameters));

        this->DisplayResult("Initialize()", mosaicInitialize (pipeline));
        // get the default reference spectrum and apply it
        this->DisplayResult("GetOutputDataFormat()", mosaicGetOutputDataFormat (pipeline, &output_data_format));
        this->DisplayResult("DeallocateCubeDataFormat()", commonDeallocateCubeDataFormat (&output_data_format));
        this->DisplayResult("Stop()", mosaicStop (pipeline));

        // re-initialize and re-start the pipeline (with reference included now)
        this->DisplayResult("SetConfigurationParameters()", mosaicSetConfigurationParameters (pipeline, parameters));
        this->DisplayResult("Initialize()", mosaicInitialize (pipeline));

        this->DisplayResult("GetOutputDataFormat()", mosaicGetOutputDataFormat (pipeline, &output_data_format));
        this->DisplayResult("Start()", mosaicStart (pipeline));

        // allocate data
        this->DisplayResult("AllocateCube()", commonAllocateCube (&data_cube, output_data_format));
    }

    ~ImageDemosaicer()
    {
        // shutdown
        mosaicPause(pipeline);
        mosaicStop(pipeline);
        commonDeallocateFrame(&data_frame);
        commonDeallocateFrameDataFormat(&data_format);
        mosaicDeallocateContext(&context);
        mosaicDestroy(&pipeline);
        system("pause");
    }

private:
    ros::Subscriber raw_sub;
    ros::Publisher cube_pub;
    int output_width;
    int output_height;
    std::string model;
    std::string package_path;
    std::string context_path;
    std::string blank_cube_path;
    HSI_RETURN ret_val;
    FrameDataFormat data_format;
    CubeDataFormat output_data_format = {};
    CubeFloat data_cube = {};
    FrameFloat data_frame = {};
    HANDLE context = nullptr;
    HANDLE pipeline = 0x0;

    std::vector<float> flattened_data;

    void cube_call_back(const sensor_msgs::Image::ConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
         Is c++   ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        ROS_INFO("CUBE RECEIVED! %d, %d", cv_ptr->image.rows, cv_ptr->image.cols);
        // TODO: DECONSTRUCT IMAGE INTO A CV::MAT, using the frame variable
        // TODO: Populate the frame from the CV::MAT
        // TODO: Verify that the PushFrame() Routine actually works here
        this->DisplayResult("PushFrame()", mosaicPushFrame (pipeline, data_frame));
        // TODO: Verify that the GetCube() Routine populates a 3D cube
        this->DisplayResult("GetCube()", mosaicGetCube (pipeline, &data_cube, 1000000));
        // TODO: Write method the takes datacube and turns it into an hyper_drive::DataCube message
        this->generate_datacube_msg();
    }

    void generate_datacube_msg()
    {
        int width = this->data_cube.format.width;
        int height = this->data_cube.format.height;
        int lambda = this->data_cube.format.nr_bands;
        std::vector<float> waves;
        std::vector<float> data_out;
        hyper_drive::DataCube msg;
        for (int curr_lam = 0; curr_lam < lambda; curr_lam++) {
            for(int row = 0; row < height; row++) {
                for(int col = 0; col < width; col++) {
                    data_out.push_back(this->data_cube.ppp_data[lambda][row][col]);
                }
            }
            // Get the wavelengths
            waves.push_back((float)this->output_data_format.band_names[curr_lam]);
        }
        msg.width = (int16_t)width;
        msg.height = (int16_t)height;
        msg.lam = (int16_t)lambda;
        msg.central_wavelengths = waves;
        // Pass the very long central data array
        msg.data = data_out;
        // TODO set the header
        //.header.stamp = ros::Time::now();
        this->cube_pub.publish(msg);
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
    ros::init(argc, argv, "cube_demosaic", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    ROS_INFO("STARTING NODE");
    ImageDemosaicer nc = ImageDemosaicer(&nh);
    ros::spin();
    return 0;
}