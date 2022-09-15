# 简易版建图/定位

主要从[autoware.ai 1.14版本core_perception](https://github.com/Autoware-AI/core_perception) **<u>抽取并重构</u>**，仅留下与mapping相关代码 较为简洁 容易部署版 拿到odom

测试系统：【注意 由于boost库限制，Ubuntu 20.04 无法运行，如想在20.04上运行 请从docker里弄 将roscore映射好就行】

- Ubuntu 18.04 ROS melodic
- Ubuntu 16.04 ROS kinetic

测试截图：

![](assets/readme/example.png)

# 使用说明

## docker

```bash
docker push zhangkin/ndt_mapping
docker run TODO
```

## 编译

```bash
mkdir -p ~/workspace/mapping_ws
cd ~/workspace/mapping_ws
git clone https://gitee.com/kin_zhang/mapping_ws.git
mv mapping_ws src
```

安装相关依赖（一些ROS包和glog）

```bash
cd src
./assets/setup_lib.sh
```

然后再编译

```bash
cd ~/workspace/mapping_ws
catkin build
source devel/setup.zsh
```

## 调参

1. 首先检查数据包有激光雷达信息，`sensor_msgs/PointCloud2` 格式

   ```bash
   rosbag info xxx.bag
   
   # ======== 示例输出 ======= topics名字可在config内修改 无需提前规定
   types:       sensor_msgs/PointCloud2 [xxx]
   topics:      /velodyne_points     5359 msgs    : sensor_msgs/PointCloud2
   ```

   打开`src/packages/lidar_localizer/config/ndt_mapping.yaml` 配置文件，修改

   ```yaml
   lidar_topic: "/velodyne_points"
   ```
   
2. 需要根据不同的建图场景进行调节，主要调节计入的最大最小距离等

   ```yaml
   # Ignore points closer than this value (meters) (default 5.0)
   min_scan_range: 1.0
   # Ignore points far than this value (meters) (default 200.0)
   max_scan_range: 50.0
   # Minimum distance between points to be added to the final map (default 1.0)
   min_add_scan_shift: 0.5
   ```

3. 如果无需建图，可开启保存一定数量的点云进行运算，把旧时刻的清除

   ```yaml
   save_frame_point: 10
   ```


在 Launch 中可以直接play bag，请修改路径即可

```bash
source ~/workspace/mapping_ws/devel/setup.zsh
roslaunch lidar_localizer ndt_mapping.launch
```

如需要保存建图结果的pcd, 请暂停bag包（因为map资源会lock住），再开一个终端并运行：

```bash
rosservice call /save_map '/home/kin/ri_dog.pcd' 0.0
rosservice call /save_map '/home/kin/ri_dog.pcd' 0.2 # save around 80% points
```

如图所示：

![](assets/readme/save_map.png)


---

**<u>博文及视频补充</u>**

相关参数介绍均在博客中进行了详细介绍：

1. [CSDN: 【Autoware】之ndt_mapping理论公式及代码对比](https://blog.csdn.net/qq_39537898/article/details/115439552#t10)

   这篇全文比较长，如果想简单使用而已，可以直接点链接看参数即可

相关使用视频：

1. [Autoware原装GUI配合使用 bilibili](https://www.bilibili.com/video/BV1k84y1F7xn)
2. [此分支安装及使用视频](TODO)

后续继续补充时，也会更新相关博文或视频进行说明



# 计划

1. 参考开源包，后续加入回环（g2o/gtsam方式）
2. 做一个建图的GUI以方便大家使用，提供安装包直接安装 无需源码编译版

