import open3d as o3d
import os

pcd_dir = '/home/ghosn/Project/uga_data_convt/test/test_30_60_lidar/'
files = os.listdir(pcd_dir)
files.sort()


idx = 1

pcd_path = os.path.join(pcd_dir, files[idx])

pcd = o3d.io.read_point_cloud(pcd_path)
o3d.visualization.draw_geometries([pcd])
