import open3d as o3d
import os

pcd_dir = '/mnt/dataset/Day2_Athens/a4_e2w/lidar'
files = os.listdir(pcd_dir)
files.sort()


idx = 2

pcd_path = os.path.join(pcd_dir, files[idx])

pcd = o3d.io.read_point_cloud(pcd_path)
o3d.visualization.draw_geometries([pcd])
