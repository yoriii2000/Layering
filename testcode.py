import open3d as o3d
import numpy as np
import math
import transformat as trans
import copy

filename = 'ScanResult_01.xyz'
berfoe_source = o3d.io.read_point_cloud(filename)
pick_point_no = trans.demo_manual_registration(berfoe_source)

f = open(filename, "r")
lines = f.readlines()
print('no = ', pick_point_no[0])

PointList = []
RawData = lines[pick_point_no[0]].strip().split()  # [x y z] from File
PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])
print('xyz = ', PointList)
