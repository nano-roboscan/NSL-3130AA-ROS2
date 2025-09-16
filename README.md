# NSL-3130AA ROS2
--- NSL-3130AA ROS2 demo ---

1. Build env
 - Ubuntu22.04.5 LTS
 - ROS2 Humble
 - OPENCV 4.5.4
 
 
2. Build NSL-3130AA demo
```
$ git clone --recurse-submodules https://github.com/nano-roboscan/NSL-3130AA-ROS2.git
$ cd NSL-3130AA-ROS2/NSL3130_driver
$ colcon build
$ . install/setup.bash
```
 
3. Start commands
```
$ ros2 launch roboscan_nsl3130 camera.Launch.py
```

# Set parameters
```
$ rqt
 (reconfigure)
```
<img width="865" height="317" alt="Image" src="https://github.com/user-attachments/assets/5775ccad-f264-418a-8981-8dea8af05ce1" />

<img width="867" height="905" alt="Image" src="https://github.com/user-attachments/assets/07616bf5-de3e-4524-9818-23b8053ada15" />
 
 
