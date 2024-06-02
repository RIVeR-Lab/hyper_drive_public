# hyper_drive
### Introduction to hyper_drive.py and cube_data.py
The main two code files in this repository are hyper_drive.py, and cube_data.py, along with their C++ counter parts grab_datacube.cpp and demosaic_datacube.cpp. hyper_drive.py is responsible for publishing images taken by the imec and ximea cameras,
and as well as for subscribing to these published images. cube_data.py also contains a publisher, and a subscriber, but instead of images, for data cubes from imec and ximea cameras.

### Launch Files
The main launch file for this repository is called master.launch which can be found in the launch folder. master.launch calls upon cube_data.py with the variables of imec or ximea, and based off the arguments, run their respective version of cube_data.py which results in a rviz live image of either the ximea or imec camera, or both. This launch file can be called by writing:
> roslaunch hyper_drive master.launch

### Camera parameter adjustments
The camera parameters can be adjusted through use of a rosservice called /adjust_param. Currently the only supported parameter adjustment is exposure time in milliseconds
When running the code, you can check to see if the srv is working by running:
> rosservice list

And you should see:
> /adjust_param

Then the value can then be adjusted by the command:
> rosservice call /adjust_param "user_in: (val)"

### Choosing camera model
The camera model can be changed by calling upon a parameter called /cube_processor/camera_model for cube_data.py, and /hsi_processor/camera_model for hyper_drive.py.
These can be changed to a string of either "imec" or "ximea" based off which camera you want the code to extract data from. 
This srv can be started by first running:

> roslaunch hyper_drive camera.launch 

This starts the launch file where the user_in input can then be adjusted.
You can check to see if the params are now visible by running:
> rosparam list

Where you should see 
> /cube_processor/camera_model

> /hsi_processor/camera_model

You can check the values in the params by running
> rosparam get (param)

And the value of the parameter can be set by running
> rosparam set (param) (val)

### Chain-running code
There are options to also run multiple programs at the same time through the use of two launch files, master.launch (code written in python), and master_opt.launch (code written using C++). The first will run cube_data.py which produces a raw image that feeds into demosaic_cube.py which then produces the final data cube, which is then passed to a PyQt GUI that allows the user to parse through the data cube by slicing the images by light channel value. master_opt.cpp does the same but using C++ code, and opens a custom Rviz config to allow viewing of the collected images. GUI will be implemented for master_opt.cpp later. These launch files can be run by:

> roslaunch hyper_drive (master or master_opt).launch

### Using ros_numpy
If you wish to write a Python script using ros_numpy, clone and build the following ROS package: https://github.com/eric-wieser/ros_numpy