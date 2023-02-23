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
hdr_mode 1 : Temperal HDR
hdr_mode 2 : Spatial HDR

imageType 0 : Grayscale 
imageType 1 : Distance 
imageType 2 : Distance / Amplitude
imageType 3 : DCS
imageType 4 : Distance / Amplitude / Grayscale

int0 , int2, int3 = 0 ~ 2500(VCSEL), 0 ~ 4000(LED)

intGr = 0 ~ 2500(VCSEL), 0 ~ 4000(LED)

modIndex 0 : 24MHz
modIndex 1 : 12MHz
modIndex 2 : 6MHz
modIndex 3 : 3MHz
modIndex 4 : 1.5MHz
modIndex 5 : 0.75MHz
```

 



 
 
 
