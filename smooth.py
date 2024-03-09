import open3d as o3d
import os
import glob

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    # Showing outliers (red) and inliers (gray):
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])
    return inlier_cloud, outlier_cloud

# 資料夾路徑
folder_path = 'C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\ScanFile_o'

# 依據檔名排序
files = sorted(glob.glob(os.path.join(folder_path, 'ScanResult*')))
i = 1
# 進行處理
# for file in files:
#     print('Processing file:', file)
#     # 讀檔
#     pcd = o3d.io.read_point_cloud(file)
#
#     # Downsample the point cloud with a voxel of 0.02
#     voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
#     uni_down_pcd = pcd.uniform_down_sample(every_k_points=5)
#
#     # Statistical oulier removal
#     cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.5)
#     display_inlier_outlier(voxel_down_pcd, ind)
#
#     # Radius oulier removal
#     cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points=30, radius=1.5)
#     display_inlier_outlier(voxel_down_pcd, ind)
#
#     o3d.io.write_point_cloud("ScanFile/ScanResult_{}.xyz".format(i), voxel_down_pcd)
#     i += 1

pcd = o3d.io.read_point_cloud('C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\ScanFile_o\\topscan.xyz')

# Downsample the point cloud with a voxel of 0.02
voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
uni_down_pcd = pcd.uniform_down_sample(every_k_points=1)

# Statistical oulier removal
cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=10,std_ratio=2.5)
inlier_cloud, outlier_cloud = display_inlier_outlier(voxel_down_pcd, ind)

o3d.io.write_point_cloud("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\ScanFile\\topscan.xyz", inlier_cloud)
