This folder includes some scripts to help you process the data.

Check the scripts folder and src folder

## Build
cpp files need build
```bash
cd simple_ndt_slam/tools
mkdir build && cd build
cmake .. && make
```

## extract_pcdTbag.cpp
Goal: extract the bag which contains point cloud data and pose to pcd files, will transform the point cloud to the world frame based on the pose. And add pose to the pcd VIEWPOINT field.

We assume that the odom and pose is already sync and have same time stamp, if you don't know how to do that please check simple_ndt_slam code `pubOtherMsgs` this function.

```bash
./bag_extract_pcd /home/kin/bags/rosbag_folder/kitti/res_odom_lidar.bag /home/kin/Test_pcd
```

If you want to the raw point map, you can use the following command, set 1 as the last parameter.

```bash
./bag_extract_pcd /home/kin/bags/rosbag_folder/kitti/res_odom_lidar.bag /home/kin/Test_pcd 1
```

Want to know how to use this produced pcd? Check [DUFOMap](TODO)!

