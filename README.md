# VINS-Mono-with-IMU-NN
## About VINS-mono-with-IMU-NN
A VI SLAM system that integrates IMU neural network observations, developed based on [vins-mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono).
## IMU neural network
IMU neural network using [RNIN-VIO](https://github.com/zju3dv/rnin-vio),the pretrained-model can be downloaded from:[model](https://pan.baidu.com/s/1GE0_MqJhFjrqdAXm62eTvA)(password:gwkw).
## sample data
Because neural networks are suitable for handheld devices, we provide a data packet collected by iphone12,sample data can be downloaded from [Rosbag](https://pan.baidu.com/s/1wv5ybQAJRutmqoEmNGLPcw)(password ovci)
## Build and Run
### 1 dependencise
1.1 ROS  [ROS Installation](http://wiki.ros.org/ROS/Installation)

```
    sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```
we use [libtorch1.8.2+cu102](https://pytorch.org/get-started/locally/) for nural network inference.

1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html), remember to **make install**.
(Our testing environment: Ubuntu 18.04, ROS Melodic, OpenCV 4.5, Eigen 3.3.3) 

### 2 build
Clone the repository and catkin_make:
```
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

### 3 run


```
    roslaunch vins_estimator euroc.launch 
    roslaunch vins_estimator vins_rviz.launch
    rosbag play YOUR_PATH_TO_DATASET/MH_01_easy.bag 
```
## Reference
[1]Chen D, Wang N, Xu R, et al. Rnin-vio: Robust neural inertial navigation aided visual-inertial odometry in challenging scenes[C]//2021 IEEE International Symposium on Mixed and Augmented Reality (ISMAR). IEEE, 2021: 275-283.

[2]Qin T, Li P, Shen S. Vins-mono: A robust and versatile monocular visual-inertial state estimator[J]. IEEE Transactions on Robotics, 2018, 34(4): 1004-1020.

