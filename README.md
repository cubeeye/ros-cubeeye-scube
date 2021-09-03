# CUBE EYE camera ROS driver(S100D / S110D device)
These are packages for using S100D / S110D device with ROS
- #### S100D / S110D

Please refer to our website for detailed product specifications. [http://www.cube-eye.co.kr](http://www.cube-eye.co.kr)



## Installation Instructions

The following instructions support ROS Kinetic, on Ubuntu 16.04.

### Step 1 : Install the ROS distribution
- #### Install ROS Kinetic, on Ubuntu 16.04

### Step 2 : Install driver
- #### Create a catkin workspace
```bash
$mkdir –p ~/catkin_ws/src
$cd ~/catkin_ws/src/
Copy the driver source to the path(catkin_ws/src)
```

- #### driver build
```bash
$catkin_init_workspace
$cd ..
$catkin_make clean
$catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Usage Instructions

Connect the camera power and execute the following command

```bash
$roscore
```

```bash
$roslaunch cubeeye_scube depth_camera.launch
```

#### Topics
- /cubeeye/scube/amplitude_raw : IR Image
- /cubeeye/scube/depth_raw : Depth(Point Cloud) Image

#### Operating Test
```bash
$rqt
/cubeeye/scube/amplitude_raw, /cubeeye/scube/depth_raw
```
<p align="center"><img width=100% src="https://user-images.githubusercontent.com/18589471/104545764-ca58ce00-55f8-11eb-86a9-3bc091a1aeb9.png"/></p>

```bash
$rosrun rviz rviz
Fixed Frame : pcl
PointCloud2 : /cubeeye/scube/points
```
<p align="center"><img width=100% src="https://user-images.githubusercontent.com/18589471/104545815-de043480-55f8-11eb-8293-baa2edba664f.png"/></p>

#### Using Dynamic Reconfigure Params
```bash
$rosrun rqt_reconfigure rqt_reconfigure
```

<p align="center"><img width=100% src="https://user-images.githubusercontent.com/18589471/104545943-21f73980-55f9-11eb-9f58-08bd199008a7.png"/></p>
