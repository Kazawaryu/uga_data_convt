import sys
ROOT_DIR = '/home/ghosn/Project/uga_data_convt'
sys.path.append(ROOT_DIR)

import cv2
import rosbag
from rospy_message_converter import message_converter
from rospy_message_converter import json_message_converter
from std_msgs.msg import String
import os
import open3d as o3d
from pypcd import pypcd
import json
import tqdm
from cv_bridge import CvBridge
import numpy as np
import pandas as pd

rosbag_path = '/media/ghosn/UGA4TB/Day2_Athens/a4_e2w.bag'
save_path = '/home/ghosn/Project/uga_data_convt/test/test_30_60_lidar'
csv_path = '/home/ghosn/Project/uga_data_convt/test/test_30_60_csv'

count = 0

with rosbag.Bag(rosbag_path) as bag:
    for topic, msg, t in tqdm.tqdm(bag.read_messages(), total=bag.get_message_count()):

        if topic == '/points_raw':
            count+=1
            if count > 30 and count <= 60:
                pcd_msg = msg.data
                pcd_msg = np.frombuffer(pcd_msg, dtype=np.float32)
                pcd_msg = pcd_msg.reshape(-1, 3)

                # remove the points that distance less than 3
                # dsitance = sqrt(x^2 + y^2 + z^2)
                distance = np.sqrt(pcd_msg[:, 0]**2 + pcd_msg[:, 1]**2 + pcd_msg[:, 2]**2)
                pcd_msg = pcd_msg[distance > 0.5]




                # save the pcd_msg to a csv file
                df = pd.DataFrame(pcd_msg, columns=['x', 'y', 'z'])
                df.to_csv(os.path.join(csv_path, f'{count}.csv'), index=False)

                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(pcd_msg)

                o3d.io.write_point_cloud(os.path.join(save_path, f'{count}.pcd'), pcd)
            elif count > 60:
                print(count)
                # exit the loop
                break
            







