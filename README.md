# hyper_drive

### Launch Files
The main launch file for this repository is called master.launch which can be found in the launch folder. `synchronousCameras.launch` calls upon `synchronousCubes.py`. The launch file can be used via the following command.
> roslaunch hyper_drive synchronousCameras.launch

### Camera parameter adjustments
The camera parameters can be adjusted through use of a rosservice called /adjust_param. Currently the only supported parameter adjustments are exposure time in milliseconds and framerate in Hz.
When running the code, you can check to see if the srv is working by running:
> rosservice list

And you should see:
> /adjust_param

Then the value can then be adjusted by the command:
> rosservice call /adjust_param "user_in: (val)"