# phoxi_camera

This package enables interfacing Photoneo PhoXi 3D Scanner/Camera from ROS. 

<img src="http://www.smartroboticsys.eu/wp-content/uploads/2016/11/photoneo_scanner.png" width="640">

###Install
*phoxi_camera* package depends on several state-of-the-art libraries and latest version of g++ compiler. Script *install_prerequisities.sh* which is available in main repo folde install all packages and libraries required for successfull phoxi_camera compilation. Follow steps below to get phoxi_camera package working properly on your system: 

```
cd catkin_ws/src
git clone https://github.com/photoneo/phoxi_camera.git
cd phoxi_camera
chmod +x install_prerequisities.sh
./install_prerequisities.sh
cd ../..
catkin_make
```
###Test PhoXi ROS interface without real 3D scanner
It is possible to test PhoXi ROS interface without real hardware. 
- Start PhoXiControl application 
- Launch simple test example```roslaunch phoxi_camera phoxi_camera_test.launch```
- Application should connect to the camera and start aquiring example pointclouds
- Notice that pointcloud data are also being published on ROS topics
- Use available ROS services to control the dummy camera.

<img src="http://www.smartroboticsys.eu/wp-content/uploads/2016/11/PhoXiControl_01.jpg" width="640">

####Available ROS services
```
/phoxi_camera/connect_camera
/phoxi_camera/disconnect_camera
/phoxi_camera/get_device_list
/phoxi_camera/get_frame
/phoxi_camera/get_hardware_indentification
/phoxi_camera/get_loggers
/phoxi_camera/get_supported_capturing_modes
/phoxi_camera/is_acquiring
/phoxi_camera/is_connected
/phoxi_camera/set_logger_level
/phoxi_camera/set_parameters
/phoxi_camera/start_acquisition
/phoxi_camera/stop_acquisition
/phoxi_camera/trigger_image
/phoxi_camera_example/get_loggers
/phoxi_camera_example/set_logger_level
```

####Available ROS topics
```
/phoxi_camera/confidence_map
/phoxi_camera/normal_map
/phoxi_camera/parameter_descriptions
/phoxi_camera/parameter_updates
/phoxi_camera/pointcloud
/phoxi_camera/texture
```


###Test PhoXi ROS interface with real device
- Start PhoXiControl application 
- Connect to your device
- Run Interface node ```rosrun phoxi_camera phoxi_camera ```
- Use available ROS services to control your 3D scanner

<img src=http://www.smartroboticsys.eu/wp-content/uploads/2016/11/PhoXiControl_02.jpg width="640">

See phoxi_camera [ROS Wiki page](http://wiki.ros.org/phoxi_camera) for further details. 

