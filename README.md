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
<img width="867" height="905" alt="Image" src="https://github.com/user-attachments/assets/07616bf5-de3e-4524-9818-23b8053ada15" />
<br/>
<img width="638" height="358" alt="Image" src="https://github.com/user-attachments/assets/7bb095b6-00d2-41d6-8527-210df8b64896" />
<br/>
<img width="1846" height="944" alt="Image" src="https://github.com/user-attachments/assets/2b9dc0d2-903f-42c7-82c0-0446d083a971" />

