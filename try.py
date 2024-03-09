import open3d as o3d
def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    # o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
    #                                   zoom=0.3412,
    #                                   front=[0.4257, -0.2125, -0.8795],
    #                                   lookat=[2.6172, 2.0475, 1.532],
    #                                   up=[-0.0694, -0.9768, 0.2024])

pcd = o3d.io.read_point_cloud('C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\ScanFile_o\\topscan.xyz')

print("Downsample the point cloud with a voxel of 0.02")
voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
uni_down_pcd = pcd.uniform_down_sample(every_k_points=5)
print("Statistical oulier removal")
cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20,std_ratio=2.0)
display_inlier_outlier(voxel_down_pcd, ind)
print("Radius oulier removal")
cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points=16, radius=0.05)
display_inlier_outlier(voxel_down_pcd, ind)
o3d.io.write_point_cloud("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\ScanFile\\topscan.xyz", voxel_down_pcd)