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

# topics:      

rosbag_path = '/media/ghosn/UGA4TB/Day2_Athens/a4_w2e.bag'
save_path = '/mnt/dataset/Day2_Athens/a4_w2e/'

# create a folder for each topic: gnss
os.makedirs(os.path.join(save_path, 'gnss'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'gnss', 'BESTPOS'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'gnss', 'BESTUTM'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'gnss', 'BESTVEL'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'gnss', 'CORRIMU'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'gnss', 'INSPVAX'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'gnss', 'INSSTDEV'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'gnss', 'ODOM'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'gnss', 'TIME'), exist_ok=True)


# create a folder for each topic: camera
os.makedirs(os.path.join(save_path, 'camera'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'camera', 'Central_Camera_blurred'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'camera', 'Left_Camera_blurred'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'camera', 'RLeft_Camera_blurred'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'camera', 'RRight_Camera_blurred'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'camera', 'Rear_Camera_blurred'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'camera', 'Right_Camera_blurred'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'camera', 'camera0', 'info'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'camera', 'camera0', 'projection_matrix'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'camera', 'camera1', 'info'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'camera', 'camera1', 'projection_matrix'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'camera', 'camera2', 'info'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'camera', 'camera2', 'projection_matrix'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'camera', 'camera3', 'info'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'camera', 'camera3', 'projection_matrix'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'camera', 'camera4', 'info'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'camera', 'camera4', 'projection_matrix'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'camera', 'camera5', 'info'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'camera', 'camera5', 'projection_matrix'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'lidar'), exist_ok=True)
os.makedirs(os.path.join(save_path, 'tf'), exist_ok=True)


bridge = CvBridge()

count = 0

with rosbag.Bag(rosbag_path) as bag:
    for topic, msg, t in tqdm.tqdm(bag.read_messages(), total=bag.get_message_count()):
        #######################################################
        # GNSS
        #######################################################
        if topic == '/APaCKV/GNSSINS/bestpos':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'gnss', 'BESTPOS', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)
        elif topic == '/APaCKV/GNSSINS/bestutm':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'gnss', 'BESTUTM', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)
        elif topic == '/APaCKV/GNSSINS/bestvel':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'gnss', 'BESTVEL', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)        
        elif topic == '/APaCKV/GNSSINS/corrimu':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'gnss', 'CORRIMU', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)
        elif topic == '/APaCKV/GNSSINS/inspvax':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'gnss', 'INSPVAX', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)
        elif topic == '/APaCKV/GNSSINS/insstdev':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'gnss', 'INSSTDEV', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)
        elif topic == '/APaCKV/GNSSINS/odom':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'gnss', 'ODOM', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)
        elif topic == '/APaCKV/GNSSINS/time':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'gnss', 'TIME', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)

        #######################################################
        # CAMERA
        #######################################################
        if topic == '/Central_Camera_blurred':
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv2.imwrite(os.path.join(save_path, 'camera', 'Central_Camera_blurred', f'{t.to_nsec()}.png'), cv_img)
        elif topic == '/Left_Camera_blurred':
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv2.imwrite(os.path.join(save_path, 'camera', 'Left_Camera_blurred', f'{t.to_nsec()}.png'), cv_img)
        elif topic == '/RLeft_Camera_blurred':
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv2.imwrite(os.path.join(save_path, 'camera', 'RLeft_Camera_blurred', f'{t.to_nsec()}.png'), cv_img)
        elif topic == '/RRight_Camera_blurred':
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv2.imwrite(os.path.join(save_path, 'camera', 'RRight_Camera_blurred', f'{t.to_nsec()}.png'), cv_img)
        elif topic == '/Rear_Camera_blurred':
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv2.imwrite(os.path.join(save_path, 'camera', 'Rear_Camera_blurred', f'{t.to_nsec()}.png'), cv_img)
        elif topic == '/Right_Camera_blurred':
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv2.imwrite(os.path.join(save_path, 'camera', 'Right_Camera_blurred', f'{t.to_nsec()}.png'), cv_img)
        # camera info
        elif topic == '/camera0/camera_info':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'camera', 'camera0', 'info', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)
        elif topic == '/camera0/projection_matrix':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'camera', 'camera0', 'projection_matrix', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)
        elif topic == '/camera1/camera_info':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'camera', 'camera1', 'info', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)
        elif topic == '/camera1/projection_matrix':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'camera', 'camera1', 'projection_matrix', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)
        elif topic == '/camera2/camera_info':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'camera', 'camera2', 'info', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)
        elif topic == '/camera2/projection_matrix':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'camera', 'camera2', 'projection_matrix', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)
        elif topic == '/camera3/camera_info':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'camera', 'camera3', 'info', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)
        elif topic == '/camera3/projection_matrix':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'camera', 'camera3', 'projection_matrix', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)
        elif topic == '/camera4/camera_info':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'camera', 'camera4', 'info', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)
        elif topic == '/camera4/projection_matrix':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'camera', 'camera4', 'projection_matrix', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)
        elif topic == '/camera5/camera_info':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'camera', 'camera5', 'info', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)
        elif topic == '/camera5/projection_matrix':
            message = json_message_converter.convert_ros_message_to_json(msg)
        #######################################################
        # LIDAR
        #######################################################
        if topic == '/points_raw':
            pcd_msg = msg.data
            pcd_msg = np.frombuffer(pcd_msg, dtype=np.float32)
            pcd_msg = pcd_msg.reshape(-1, 3)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pcd_msg)
            o3d.io.write_point_cloud(os.path.join(save_path, 'lidar', f'{t.to_nsec()}.pcd'), pcd)

            # TODO: fix the beam noise points
            count+=1
            if count == 30:
                # save the msg  to a temp
                temp_save = './temp.txt'
                with open(temp_save, 'w') as f:
                    # to string
                    msg_str = str(msg)
                    f.write(msg_str)
                break
        #######################################################
        # TF
        #######################################################
        elif topic == '/tf':
            message = json_message_converter.convert_ros_message_to_json(msg)
            with open(os.path.join(save_path, 'tf', f'{t.to_nsec()}.json'), 'w') as f:
                json.dump(message, f)
            