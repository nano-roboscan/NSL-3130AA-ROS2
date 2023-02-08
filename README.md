# NSL-3130AA ROS2
--- NSL-3130AA ROS2 demo ---

1. Build env
 - Ubuntu22.04.1 LTS
 - ROS2 Humble
 - OPENCV 4.5.4
 
 
2. Build NSL-3130AA demo
 - cd NSL3130_driver
 - colcon build --packages-select roboscan_nsl3130
 - . install/setup.bash
 
 
3. Start commands
 - ros2 run roboscan_nsl3130 roboscan_publish_node
 - ros2 launch roboscan_nsl3130 camera.Launch.py

# Set parameters
 $rqt
 (reconfigure)
 
 
 --- view ---
 
 ![ROS2](https://user-images.githubusercontent.com/106071093/215962627-2f14e35d-c8e9-4d01-a675-e5ee0ee13fb4.png)

 
 
 
 
 
