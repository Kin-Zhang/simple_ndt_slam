'''
 * Copyright (C) 2022, RPL, KTH Royal Institute of Technology
 * MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2022-12-02
 * Description: Extract the result bag for erasor package to settle down, 
 check how to erasor the dynamics points in the map: https://github.com/Kin-Zhang/ERASOR/tree/simple_ndt_slam
'''


import numpy as np
import rosbag
import os, argparse, time
import open3d
from pathlib import Path
import csv
import roslaunch


parser = argparse.ArgumentParser()
parser.add_argument('--bag-path', default='/home/kin/bags/autoware/res_three_people_back_xuzhou.bag')
parser.add_argument('--save-dir', default='/home/kin/bags/autoware/sub_three_people_back_xuzhou')
parser.add_argument('--odom',  default='/auto_odom')
parser.add_argument('--lidar', default='/odom_lidar')
args = parser.parse_args()

if __name__ == '__main__':
    start_time = time.time()

    extract_bag = rosbag.Bag(f'{args.bag_path}', 'r')
    pcd_folder = f"{args.save_dir}/pcds"
    Path(pcd_folder).mkdir(exist_ok=True, parents=True)
    print(args)

    # extract pose
    poses = ""
    pid = 0
    for topic, msg, t in extract_bag.read_messages(topics=[args.odom]):
        timestamp = t.to_time()
        if topic==args.odom:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            poses = poses + "{}, {:.8f}, {:.8f}, {:.8f}, {:.8f}, {:.8f}, {:.8f}, {:.8f}, {:.8f}\n".format(pid, timestamp, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w)
            pid = pid + 1 #

    # extract pcd
    if len(os.listdir(pcd_folder)) == 0:
        uuid = roslaunch.rlutil.get_or_generate_uuid(options_runid=None, options_wait_for_master=False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_files=[], is_core=True)
        launch.start()
        cmd = f"rosrun pcl_ros bag_to_pcd {args.bag_path} {args.lidar} {pcd_folder}"
        os.popen(cmd).readlines()
        #  --- your code ---
        launch.shutdown()
        # Absolute path of a file
        all_pcds = os.listdir(pcd_folder)
        all_pcds.sort()
        for i, path in enumerate(all_pcds):
            old_name = f"{pcd_folder}/{path}"
            new_name = "{}/{:06d}.pcd".format(pcd_folder, i)
            # Renaming the file
            os.rename(old_name, new_name)

    with open(f'{args.save_dir}/poses_lidar2body.csv', 'w') as f:
        f.write("index, timestamp, x, y, z, qx, qy, qz, qw\n")
        f.write(poses)
    print(f"finished!!! total time: {(time.time()-start_time)/60:.2f} mins")
    print(f"\033[92m CHECK YOUR DATASET RESULT ON {args.save_dir} \033[0m")