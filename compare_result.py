import File
import open3d as o3d
import os as os
import glob
import copy

import os as os
import open3d as o3d
import numpy as np
import math
from numpy.linalg import inv
from sklearn.neighbors import KDTree

import Computelayer as rest_depth



def Sq2(value):
    # Code untuk Kuadrat bilangan
    return value*value

def SaveFile(Pcd_File_Name, PCDList):

    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])




check_before = o3d.io.read_point_cloud("60degree.xyz")
target = o3d.io.read_point_cloud("ScanFile/ScanResult_2.xyz")
restcheckfiel = 'Source File/60degree_2.xyz'
Ref_File = 'cluster/refCluster3.xyz'
icptran_text = 'test/60tran.xyz'

threshold = 0.3
trans_init = np.asarray([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
draw_registration_result(check_before, target, trans_init)

reg_p2p = o3d.pipelines.registration.registration_icp(check_before, target, threshold, trans_init,
                                                      o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                      o3d.pipelines.registration.ICPConvergenceCriteria(
                                                          max_iteration=2500))

trans = reg_p2p.transformation
np.savetxt(icptran_text, trans, fmt='%0.10g', delimiter=' ')

draw_registration_result(check_before, target, reg_p2p.transformation)
print(reg_p2p.transformation)


o_icp = 'test/60tran.xyz'
o_tran = 'test/eTc.xyz'
tr = np.array([[1, 0, 0, 0], [0, 1, 0, -0.38], [0, 0, 1, 0], [0, 0, 0, 1]])
first_tran = 'test/1_pose_tran_1.xyz'
two_tran = 'test/tran_1.xyz'

ori = File.Readmatricfile(o_icp)
sTe = File.Readmatricfile(o_tran)
ro_tra = File.Readmatricfile(first_tran)
icp_tra = File.Readmatricfile(two_tran)

o = np.asarray(ori)
s = np.asarray(sTe)
ro = np.asarray(ro_tra)
icp = np.asarray(icp_tra)

beginpose = check_before.transform(o)
check_after = beginpose.transform(s)
check_after_1 = check_after.transform(tr)
check_after_2 = check_after_1.transform(ro)
check_after_4 = check_after_2.transform(icp)

o3d.io.write_point_cloud(restcheckfiel, check_after_4)

Pcd = File.ReadXyzFile(restcheckfiel)
for PcdNo in range(0, len(Pcd)):
    Pcd[PcdNo].append(0)
    Pcd[PcdNo].append(0)
    Pcd[PcdNo].append(0)

# File.SaveFile('Source File/RemovedPoints_9', Pcd)
File.SaveFile('Source File/60degree_2', Pcd)

Layer_height = 0.05
Vertice2 = File.ReadXyzFile(restcheckfiel)
Vertice2 = np.asarray(Vertice2)
tree = KDTree(Vertice2, leaf_size=8)
NewPoint, NewPointIdx, NewNormal = File.ReadXYZNormalFile(Ref_File)
rest_depth.Layering(NewPoint, NewPointIdx, NewNormal, tree, Vertice2, LayerDepth=Layer_height)
