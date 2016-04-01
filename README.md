# phoxi_camera
if you have not catkin workspace:
```
mkdir -p ~/catkin_workspace_directory/src
cd ~/catkin_workspace_directory/src
catkin_init_workspace
```
then:
```
mkdir -p ~/catkin_workspace_directory/src/phoxi_camera
cd ~/catkin_workspace_directory/src/phoxi_camera
git init
git pull https://github.com/photoneo/phoxi_camera.git
cd ../..
catkin_make
source ~/catkin_workspace_directory/devel/setup.bash
catkin_make
rosrun phoxi_camera phoxi_camera_node
rosrun phoxi_camera phoxi_camera_example.py
```
In phoxi_camera_example.py change hardware identification of camera
