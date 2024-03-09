import numpy as np
import os as os
import glob
import shutil
import re

def ReadXyzFile(filename):
    # print('File Path:', filename)
    f = open(filename, "r")
    lines = f.readlines()
    # print('No of Points [XYZ]:', len(lines))
    PointList = []

    for x in range(0, len(lines)):
        RawData = lines[x].strip().split() #[x y z] from File
        PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

    return PointList
def SaveFile(Pcd_File_Name, PCDList):
    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))

point = []
# 開啟檔案並讀取內容
with open('Res_extrudepart.txt', 'r') as f:
    line = f.readline()
    protrusion, traj_count = map(int, line.split())
print('須新增層數: ', traj_count)

# 從 'Res_extrudepart.txt' 讀取凸點編號以及需要新增的軌跡的條數
with open('Res_extrudepart.txt', 'r') as f:
    line = f.readline()
    protrusion, traj_count = map(int, line.split())

print('凸點編號 = ', protrusion)
Layer_num = sorted(glob.glob(os.path.join("Output File/", "*traj{}*".format(protrusion))), key=lambda x: (int(re.split('traj|_|_|.xyz', x)[1])))
Layer_number = len(Layer_num)
Layer = Layer_number-6
print('{} 號凸點原本的層數 = '.format(protrusion), Layer)

Layer_file = sorted(glob.glob(os.path.join("Output File/", "*traj{}_{}*".format(protrusion, Layer-2))), key=lambda x: (int(re.split('traj|_|_|.xyz', x)[1])))  # 倒數第二層
tra_0 = [ReadXyzFile(file) for file in Layer_file]
# print('0-tra_0 = ', tra_0)
Layer_file = sorted(glob.glob(os.path.join("Output File/", "*traj{}_{}*".format(protrusion, Layer-1))), key=lambda x: (int(re.split('traj|_|_|.xyz', x)[1])))  # 最後一層
tra_1 = [ReadXyzFile(file) for file in Layer_file]
# print('0-tra_1 = ', tra_1)

for tra_No in range(0, traj_count):
    if tra_No == 0:
        for j in range(0, 3):
            new_trajectory = np.array(tra_1[j]) - (np.array(tra_0[j]) - np.array(tra_1[j]))
            SaveFile('inc_trajectory/inctra{}_{}_{}.xyz'.format(protrusion, tra_No, j), new_trajectory)
    elif tra_No == 1:
        inctra_file = sorted(glob.glob(os.path.join("inc_trajectory/", "*inctra{}_{}*".format(protrusion, tra_No - 1))),
                            key=lambda x: (int(re.split('inctra|_|_|.xyz', x)[2])))
        # print('inctra_file', inctra_file)
        tra_0 = tra_1
        tra_1 = [ReadXyzFile(file) for file in inctra_file]
        # print('2-tra_0', tra_0)
        # print('2-tra_1', tra_1)
        for j in range(0, 3):
            new_trajectory = np.array(tra_1[j]) - (np.array(tra_0[j]) - np.array(tra_1[j]))
            SaveFile('inc_trajectory/inctra{}_{}_{}.xyz'.format(protrusion, tra_No, j), new_trajectory)
    elif tra_No >= 2:
        inctra_file_last2layers = sorted(glob.glob(os.path.join("inc_trajectory/", "*inctra{}_{}*".format(protrusion, tra_No - 2))),
                                        key=lambda x: (int(re.split('inctra|_|_|.xyz', x)[2])))
        inctra_file_last1layers = sorted(glob.glob(os.path.join("inc_trajectory/", "*inctra{}_{}*".format(protrusion, tra_No - 1))),
                                        key=lambda x: (int(re.split('inctra|_|_|.xyz', x)[2])))
        tra_0 = [ReadXyzFile(file) for file in inctra_file_last2layers]
        tra_1 = [ReadXyzFile(file) for file in inctra_file_last1layers]
        # print('3-tra_0', tra_0)
        # print('3-tra_1', tra_1)
        for j in range(0, 3):
            new_trajectory = np.array(tra_1[j]) - (np.array(tra_0[j]) - np.array(tra_1[j]))
            SaveFile('inc_trajectory/inctra{}_{}_{}.xyz'.format(protrusion, tra_No, j), new_trajectory)

