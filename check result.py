import math
import scipy.linalg
import numpy as np
import open3d as o3d
import os as os

# import transformat as trans


import glob
import shutil
import re
import matplotlib.pyplot as plt

def ReadXyzFile(filename):
    print('File Path:', filename)
    f = open(filename, "r")
    lines = f.readlines()
    print('No of Points [XYZ]:', len(lines))
    PointList = []

    for x in range(0, len(lines)):
        RawData = lines[x].strip().split() #[x y z] from File
        # print('r = ', len(RawData))
        if len(RawData) > 3:
            PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2]), float(RawData[3]), float(RawData[4]), float(RawData[5])])
        else:
            PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

    return PointList

def SaveFile(Pcd_File_Name, PCDList):
    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))


#
#
# movematric = np.array([[1, 0, 0, 0], [0, 1, 0, 0.5], [0, 0, 1, 0], [0, 0, 0, 1]])
# # movematric = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
#
# bmetalmodel = o3d.io.read_point_cloud("test/metalmodel.xyz")
# metalmodelcentor = bmetalmodel.transform(movematric)
# o3d.io.write_point_cloud("test/metalmodelcentor.xyz", metalmodelcentor)

# input()
plt.figure(figsize=(8, 4), dpi=95)

plt.suptitle('Force Data', fontsize=20)
plt.grid()

a = sorted(glob.glob(os.path.join("force data/", "F_line0*")))
print(a)
# input()
X = []
Y = []
for file0_no in range(0, len(a)):
    filename = a[file0_no]

    file = open(filename, 'r')  # 1
    lines = file.readlines()  # 2
    # print('Data = ', len(lines))
    for no in range(0, len(lines)):  # 3
        # print('line', lines[no])
        value = re.split(' s| \t| |N', lines[no])
        # value = [float(s) for s in lines[no].split()]  # 4
        # print('va', value)
        if float(value[0]) > 1:

            X.append(float(value[0]))  # 5
            Y.append(float(value[1]))


    # print(X)
    # print(Y)
plt.scatter(X, Y, color='tab:red')
plt.savefig("force data/total_Force")

plt.show()

input()

# result_file = 'cluster/rawCluster0.xyz'
# compute_file = 'cluster/refCluster0.xyz'
# result = ReadXyzFile(result_file)
# compute_result = ReadXyzFile(compute_file)
# detect = result + compute_result
# twoD_result = []
# for i in range(0, len(detect)):
#     if -0.5 < detect[i][0] < 0.5:
#         twoD_result.append([detect[i][0], detect[i][1], detect[i][2]])
#
# SaveFile('twoD_result.xyz', twoD_result)
# print(result)

