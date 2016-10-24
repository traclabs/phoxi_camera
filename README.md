# phoxi_camera

ROS Package for PhoXi Scanner/Camera

if you don't have catkin workspace:
```
mkdir -p ~/catkin_workspace_directory/src
cd ~/catkin_workspace_directory/src
catkin_init_workspace
```
then:
```
cd ~/catkin_workspace_directory/src
git clone https://github.com/photoneo/phoxi_camera.git
cd ..
catkin_make
source ~/catkin_workspace_directory/devel/setup.bash
rosrun phoxi_camera phoxi_camera_node
rosrun phoxi_camera phoxi_camera_example.py
```
In phoxi_camera_example.py change hardware identification of camera

Generate package:
```
catkin_generate_changelog
catkin_prepare_release
bloom-release --rosdistro <ros_distro> --track <ros_distro> phoxi_camera
```
