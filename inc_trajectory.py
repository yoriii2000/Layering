import numpy as np
import os as os
import glob
import shutil
import re

def ReadXyzFile(filename):
    print('File Path:', filename)
    f = open(filename, "r")
    lines = f.readlines()
    print('No of Points [XYZ]:', len(lines))
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
    # 讀取整行，並使用 split() 方法以空格為分隔符將字符串分裂成列表
    content = f.readline().split()

# 將讀取到的字串轉換為浮點數
No_extrudepart = int(content[0])
Res_extrudepart = float(content[1])
print('剩餘凸點高度(mm): ', Res_extrudepart)
print('新增層數(mm): ', round(Res_extrudepart/0.2))


trajectory_num = sorted(glob.glob(os.path.join("Output File/", "*traj*")), key=lambda x: (int(re.split('traj|_|_|.xyz', x)[1])))
print(len(trajectory_num))
protrusion_number = int(re.split('|_|_|.xyz', trajectory_num[len(trajectory_num)-1])[17]) + 1
print('凸點數量 = ', protrusion_number)
for protruison_no in range(0, protrusion_number):
    Layer_num = sorted(glob.glob(os.path.join("Output File/", "*traj{}*".format(protruison_no))), key=lambda x: (int(re.split('traj|_|_|.xyz', x)[1])))
    Layer_number = len(Layer_num)
    print('層數 = ', Layer_number-6)
    layers = Layer_number-6
    i = -1
    newtra = []
    tra = []
    inc_tra = []
    for Layer_no in range(0, layers):
        Layer_file = sorted(glob.glob(os.path.join("Output File/", "*traj{}_{}*".format(protruison_no, Layer_no))), key=lambda x: (int(re.split('traj|_|_|.xyz', x)[1])))
        i = i + 1
        # print('i = ', i)
        # print('layers = ', layers)
        k = -1
        tra.append([None] * 3)
    #     print(Layer_file)
    #     print(len(Layer_file))
        last2layers = sorted(glob.glob(os.path.join("Output File/", "*traj{}_{}*".format(protruison_no, i))), key=lambda x: (int(re.split('traj|_|_|.xyz', x)[1])))
        if i == layers - 2:  # 倒數第二層
            # print(last2layers)
            k = k + 1  # 計算層數
            # if len(tra) <= k:
            #     tra.append([None] * 3)
            for j in range(0, 3):  # 第幾條軌跡
                # print('j', j)
                # print('k', k)
                # print('last2layers[j] = ', last2layers[j])
                tra[0][j] = ReadXyzFile(last2layers[j])
                # print('tra[k][j] = ', tra[k][j])
            # print('tra[0][:] = ', tra[0][:])
            inc_tra.append(tra[0][:])
            # print('tra[0][0] = ', tra[0][0])
            # print('tra[0][1] = ', tra[0][1])
            # print('tra[0][2] = ', tra[0][2])
        if i == layers - 1:  # 最後一層
            k = k + 1
            tra.append([None] * 3)
            for j in range(0, 3):
                tra[1][j] = ReadXyzFile(last2layers[j])
                # print('tra[k][j] = ', tra[k][j])
            # print('tra[1][:] = ', tra[1][:])
            inc_tra.append(tra[1][:])
            # print('tra[1][0] = ', tra[1][0])
            # print('tra[1][1] = ', tra[1][1])
            # print('tra[1][2] = ', tra[1][2])
            for j in range(0, 3):
                print('tra[0][j] = ', tra[0][0])
                print('tra[1][j] = ', tra[1][0])
                newtra = np.array(tra[1][j]) - (np.array(tra[0][j]) - np.array(tra[1][j]))
                SaveFile('inc_trajectory/traj{}_{}_{}.xyz'.format(protruison_no, Layer_no, j), newtra)

        # input('stop')