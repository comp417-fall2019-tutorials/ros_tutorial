# ROS Tutorial 1 

## Launching World File 

__Build Project__ 
 ```shell script
catkin_make 
 ```

__Run Launch File__ 
```shell script
roslaunch ros_tutorial world.launch 
```  

![environment](./documentation/outdoor_environment.png)

## Keyboard Control of Car
- The keyboard_control node can be used to manually control the robot agent.
```
roslaunch ros_tutorial keyboard_control.launch 
``` 

![environment](./documentation/keyboard_control.png)
 
## Utilities

__RQT Image View__ 
- Launch RQT image view and subscribe to camera/iamge_raw
```shell script
rqt_image_view
```
![environment](./documentation/rqt_image_view.png)
 
__RVIZ__
```shell script
rviz 
```

![environment](./documentation/rviz.png)



 

