# phoxi_camera
if you have not catkin workspace:
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
bloom-release foo --track indigo --rosdistro indigo
```
