This folder includes some scripts to help you process the data.

Check the scripts folder and src folder

## Build
cpp files need build
```bash
cd simple_ndt_slam/tools
mkdir build && cd build
cmake .. && make
```

## extract_pcdFbag_*.cpp
Goal: extract the bag which contains point cloud data and pose to pcd files, will transform the point cloud to the world frame based on the pose. And add pose to the pcd VIEWPOINT field.

We assume that the odom and pose is already sync and have same time stamp, if you don't know how to do that please check simple_ndt_slam code `pubOtherMsgs` this function. If you are using other slam package and want to use this script, you need to sync the odom and pose by yourself or tf need be tf2, [check why tf2 instead of tf here](../assets/readme/WHY-TF2.md)

- `_tf`: directly read pose msg from tf2
- `_topic`: need sync pose and then specify the odom topic name.

### Usage

Command:
```bash
./bag2pcd_tf <rosbag_path> <save_pcd_folder> <pc2_topic_name> <world_or_map_frame_id> [optional: set 1 to save raw map]

```

```bash
# example
./bag2pcd_tf /home/kin/bags/kobuki/res_0425vlp2livox.bag /home/kin/Tmp/haupt_mp /points_raw map
```

And here is the folder tree:
```
➜  Tmp tree -L 2
.
├── pcd
│   ├── 000000.pcd
│   ├── 000001.pcd
│   ├── 000002.pcd
│   ├── 000003.pcd
│   ├── ...
│   ├── 000667.pcd
│   ├── 000668.pcd
│   └── 000669.pcd
└── raw_map.pcd

```

Want to know how to use this produced pcd? Check [DUFOMap](TODO)!

