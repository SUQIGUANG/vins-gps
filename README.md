# VINS-GPS
此项目可作为一个VINS教学版，里面加入了大量本人对VINS的一些理解注释。除此之外，本项目将在VINS的基础上新增Visual+IMU+GPS耦合的多传感器融合demo，与VINS其他demo一样，本项目也运行在ROS上。

## 1.预装项

### 1.1 系统

在Ubuntu18.04 LTS中测试，预计Ubuntu16.04 LTS中也能正常运行(Ubuntu20 兼容性暂时未知)。

安装对应的ROS版本( Ubuntu16.04 LTS对应ROS Kinetic；Ubuntu18.04 LTS对应ROS Melodic )，[ROS 安装教程](http://wiki.ros.org/ROS/Installation)。

### 1.2 Ceres安装

[Ceres 安装教程](http://ceres-solver.org/installation.html)

## 2. 编译项目

```
    cd ~/catkin_ws/src
    git clone 
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Test

### 3.1 Test with KITTI

- KITTI Stereo

下载[KITTI Odometry dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) ，以sequences 00为例

1. kitti  stereo+gps

```bash
# 终端1
roscore

# 终端2
source catkin_ws/devel/setup.bash
roslaunch vins vins_rviz.launch

# 终端3
source catkin_ws/devel/setup.bash
rosrun vins kitti_gps_test /home/sqg/catkin_ws/src/vins-gps/config/kitti_raw/kitti_10_03_config.yaml /home/sqg/dataset/kitti/2011_10_03_drive_0027_sync

# 终端4
source catkin_ws/devel/setup.bash
rosrun global_fusion global_fusion_node
```

2. kitti  stereo

```bash
# 终端1
roscore

# 终端2
source catkin_ws/devel/setup.bash
roslaunch vins vins_rviz.launch

# 终端３
source catkin_ws/devel/setup.bash
rosrun vins kitti_odom_test /home/sqg/catkin_ws/src/vins-gps/config/kitti_odom/kitti_config00-02.yaml /home/sqg/dataset/kitti/00

# 终端４ (可选)
source catkin_ws/devel/setup.bash
rosrun loop_fusion loop_fusion_node /home/sqg/catkin_ws/src/vins-gps/config/kitti_odom/kitti_config00-02.yaml 
```

3. kitti stereo+IMU

```bash
# 终端1
roscore

# 终端2
source catkin_ws/devel/setup.bash
roslaunch vins vins_rviz.launch

# 终端3
source catkin_ws/devel/setup.bash
rosrun vins kitti_vins_test /home/sqg/catkin_ws/src/vins-gps/config/kitti_raw/kitti_10_03_config_imu.yaml /home/sqg/dataset/kitti/2011_10_03_drive_0027_sync

# 终端4
source catkin_ws/devel/setup.bash
rosrun global_fusion global_fusion_node
```

4. euroc mono+IMU

```bash
# 终端1
roscore

# 终端2
source catkin_ws/devel/setup.bash
roslaunch vins vins_rviz.launch

# 终端3
source catkin_ws/devel/setup.bash
rosrun vins vins_node /home/sqg/catkin_ws/src/vins-gps/config/euroc/euroc_mono_imu_config.yaml 

# 终端4（可选）
source catkin_ws/devel/setup.bash
rosrun loop_fusion loop_fusion_node /home/sqg/catkin_ws/src/vins-gps/config/euroc/euroc_mono_imu_config.yaml

# 终端5
source catkin_ws/devel/setup.bash
rosbag play ~/dataset/euroc/MH_01_easy.bag
```

5. euroc stereo+IMU

6. euroc stereo

- KITTTI Stereo+GPS+IMU



### 3.2 Test with your own dataset



下一步工作，污染GPS数据，做鲁棒估计



> 感谢香港科技大学[Aerial Robotics Group](http://uav.ust.hk/) [秦通](http://www.qintonguav.com)等人的VINS开源项目