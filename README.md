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

# NSL-3130AA View

  ![ROS2](https://user-images.githubusercontent.com/106071093/218378867-6792ac7b-4b2f-4227-9fa3-ef833f0fc784.png)

# Rollover Avoidance and Correction

  ![rollover](https://github.com/nano-roboscan/NSL-3130AA-ROS2/assets/106071093/3d30e77f-0b82-48d1-be3e-2ca3455a9bba)
  ![roll-over-gif2](https://github.com/nano-roboscan/NSL-3130AA-ROS2/assets/106071093/b3a04a07-6822-4076-8dc5-69eed91bc935)

# Average FPS per Image type

  ![fps](https://github.com/nano-roboscan/NSL-3130AA-ROS2/assets/106071093/532178f4-23ac-4eee-ae8d-a4f8fb03b747)


# Set parameters
```
$ rqt
 (reconfigure)
```

![Screenshot from 2023-02-22 13-00-21 (copy)](https://user-images.githubusercontent.com/106071093/220520356-3d16736f-902c-4d9e-858d-a6ed8ddf87aa.png)


```

cvShow : Image Viewer on/off
channel : 0 ~ 15

hdr_mode 0 : HDR off
hdr_mode 1 : Spatial HDR
hdr_mode 2 : Temperal HDR

imageType 0 : Grayscale 
imageType 1 : Distance 
imageType 2 : Distance / Amplitude
imageType 3 : DCS
imageType 4 : Distance / Grayscale
imageType 5 : Distance / Amplitude / Grayscale

int0 , int2, int3 = 0 ~ 2500(VCSEL), 0 ~ 4000(LED)

intGr = 0 ~ 2500(VCSEL), 0 ~ 4000(LED)

modIndex 0 : 24MHz
modIndex 1 : 12MHz
modIndex 2 : 6MHz
modIndex 3 : 3MHz


transformAngle : angle (rviz-based y-axis rotation)
```

 



 
 
 
