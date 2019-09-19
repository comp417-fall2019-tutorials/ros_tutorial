# Environment Setup
- Source the ros setup script:

```shell script
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

- Make your catkin workspace: 

```shell script
cd ~
mkdir -p catkin_ws/src
``` 

- Place package code in the catkin_ws/src folder. We will use this repository for that purpose 

```shell script
cd catkin_ws/src
git clone https://github.com/stefanwapnick/ros_tutorial.git
```

- Build your catkin_ws folder. Navigate back to the catkin_ws folder and execute the `catkin_make` command 
```shell script
cd ../
catkin_make
```

- Full installation instructions can be found here: [ROS Melodic Installation Guide](http://wiki.ros.org/melodic/Installation/Ubuntu)

# Simple Publisher Subscriber Example
- Launch the publisher and subscriber node

```shell script
roslaunch ros_tutorial pub_sub.launch
``` 

__Expected output:__ 

![pub_sub_output](documentation/pub_sub_output.png)

- Some additional commands to try: 
```shell script
# Show list of runnign topics
rostopic list 
# See how fast a topic is publishing 
rostopic hz /msg 
# Echo the value of a topic 
rostopic echo /msg
``` 

# Keyboard Controlled Car

- Run the launch file to create the gazebo environment
 
```shell script
roslaunch ros_tutorial world.launch 
```  

![environment](./documentation/outdoor_environment.png)

- The keyboard_control node can be used to manually control the robot agent.
- Try moving the agent around using the keyboard commands
```
roslaunch ros_tutorial keyboard_control.launch 
``` 

![environment](./documentation/keyboard_control.png)
 
# Utilities

__RQT Image View__ 
- Launch RQT image view and subscribe to camera/iamge_raw
```shell script
rqt_image_view
```
![environment](./documentation/rqt_image_view.png)
 
__RVIZ__
- Open RVIZ from the console with the following command
```shell script
rviz 
```

![environment](./documentation/rviz.png)

- Click on the __add__ button and select topics to visualize 
- For example, select LaserScan to visualize the LIDAR returns

![rviz_item_selection](./documentation/rviz_item_selection.png)

- After the LaserScan option has been added, make sure that it points to the right topic: __/scan__

![scan_topic_selection](./documentation/scan_topic_selection.png)





 

