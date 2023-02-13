# NSL-3130AA ROS2
--- NSL-3130AA ROS2 demo ---

1. Build env
 - Ubuntu22.04.1 LTS
 - ROS2 Humble
 - OPENCV 4.5.4
 
 
2. Build NSL-3130AA demo
```
$ cd NSL3130_driver
$ colcon build --packages-select roboscan_nsl3130
$ . install/setup.bash
```
 
3. Start commands
```
$ ros2 run roboscan_nsl3130 roboscan_publish_node
$ ros2 launch roboscan_nsl3130 camera.Launch.py
```

# Set parameters
```
$ rqt
 (reconfigure)
```
 
 
# NSL-3130AA View


  ![ROS2](https://user-images.githubusercontent.com/106071093/218378867-6792ac7b-4b2f-4227-9fa3-ef833f0fc784.png)




 
 
 
