# Robot-Programming
Robot Programming project - *Simple RVIz*

The project involves the control of a mobile robot in ROS and its visualization in **RVIz**.
Specifically, the following nodes have been implemented:
- a *mobile_base* node, to publish the odometry of the robot (a unicycle)
- a *laser_scan* node, to publish the laser data
- a *ipose* node, to display the initial and goal poses of the robot

Finally, to localize the robot and plan its path, the **Nav2** software was used.

## How to compile
The project is developed in **Ros2-Humble** and **Nav2** to implement the localization.

### Requirements

Install Nav2 packages:
```
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```
### Cloning and setting the workspace
Create a workspace
```
mkdir -p ~/ros2_ws/src
```
in which cloning the following repository:
```
cd ~/ros2_ws/src
git clone https://github.com/CatiaRomaniello/Robot-Programming.git
```
And finally compiling and sourcing it with
```
colcon build
source install/setup.bash
```
## How to run
Run the launch file 
```
ros2 launch navigation_ mapping_launch.py 
```
and a screen will appear with RVIz and all the nodes already started.
At this point, you need to set an initial pose by selecting the **2D Pose Estimate** button and choosing a point on the map; 
the robot will appear at the chosen position. 
Finally, to see it in motion, you need to choose another point on the map with the **2D Goal** button.

![Screencastfrom09-25-2024041002PM-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/e8f37cb4-d2f6-4693-bb80-34cfed3317fb)
![Screencastfrom09-25-2024035056PM-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/149ab249-df49-4669-9b9e-43db3fff370e)
