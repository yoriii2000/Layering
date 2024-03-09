import File
import open3d as o3d
import os as os
import glob

import os as os
import open3d as o3d
import numpy as np
import math
from numpy.linalg import inv

import time
import shutil
import re

def Sq2(value):
    # Code untuk Kuadrat bilangan
    return value*value

def SaveFile(Pcd_File_Name, PCDList):

    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))


def ReadXyzNorFile(filename):
    print('File Path:   ', filename)
    f = open(filename, "r")
    lines = f.readlines()
    # print('No of Points [XYZ]:', len(lines))
    PointList = []

    for x in range(0, len(lines)):
        RawData = lines[x].strip().split() #[x y z] from File
        # print('r = ', len(RawData))
        if len(RawData) > 3:
            PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2]), float(RawData[3])])
        else:
            PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

    return PointList


# protrusion_numaber = 3
a = sorted(glob.glob(os.path.join("each layer/", "*.xyz")), key=lambda x: (int(re.split('Cluste_|_|.xyz', x)[1]),
                                                                           int(re.split('Cluste_|_|.xyz', x)[2])))
print(a)
protrusion_numaber = int(re.split('_|.xyz', a[len(a) - 1])[1]) + 1
print('total protrusion = ', protrusion_numaber)
robot_error = float(input('粗磨手臂誤差 = '))   # 單位mm  5
robot_error_fine = float(input('細磨手臂誤差 = '))  # 單位mm  10
for protruison_no in range(0, protrusion_numaber):
    print('protruison_no = ', protruison_no)

    f0 = sorted(glob.glob(os.path.join("grindcoor{}/".format(protruison_no), "*.xyz")), key=lambda x: (
        int(re.split('eTt{}_|_|.xyz'.format(protruison_no), x)[1]),
        int(re.split('eTt{}_|_|.xyz'.format(protruison_no), x)[2]),
        int(re.split('eTt{}_|_|.xyz'.format(protruison_no), x)[1])))

    shutil.rmtree('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\rTe_matric{}'.format(protruison_no))
    os.makedirs('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\rTe_matric{}'.format(protruison_no))
    print('point_no = ', f0)
    # input()

    AllxyzRxyz0 = []
    for file0_no in range(0, len(f0)):

        # 軌跡點相對模型原點 cTt -----------------------------------------------------------------------------------------------
        transf = ReadXyzNorFile(f0[file0_no])
        eTt = np.asarray(transf)
        # print(eTt)

        tTe = inv(eTt)

        # # 模型原點相對法蘭面 eTc -----------------------------------------------------------------------------------------------
        # eTc = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, (39.45 + 144.2)], [0, 0, 0, 1]])  # code 裡單位為毫米mm  183.65
        # # eTc = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 1.3965e+02], [0, 0, 0, 1]])  # code 裡單位為毫米mm
        #
        # # print('eTC =\n', eTc)
        #
        # cTe = inv(eTc)
        # print('cTe =\n', cTe)

        # #研磨點相對手臂 rTt ----------------------------------------------------------------------------------------------
        # 砂輪研磨點位置角度 (雙頭)---

        # if file0_no < len(f0) - 30:
        #
        #     gdt = np.array([[4.0177e+02, 1.3467e+03, -1.7707e+02]])  # 粗磨研磨點
        #     # 建立研磨點坐標系
        #     gdt_o = np.array([[4.0177e+02, 1.3467e+03, -1.7707e+02]])  # 粗磨原點
        #     gdt_y = np.array([[4.2964e+02, 1.3476e+03, -1.7707e+02]])  # 粗磨y方向
        #     gdt_z = np.array([[3.9511e+02, 1.5247e+03, -1.8158e+02]])
        # else:
        #     gdt = np.array([[-4.6881e+02, 1.3299e+03, -1.7648e+02]])    # 細磨研磨點
        #     # 建立研磨點坐標系
        #     gdt_o = np.array([[-4.6881e+02, 1.3229e+03, -1.7648e+02]])   # 細磨原點
        #     gdt_y = np.array([[-4.4688e+02, 1.3235e+03, -1.7648e+02]])   # 細磨y方向
        #     gdt_z = np.array([[-4.7624e+02, 1.4932e+03, -1.8535e+02]])   # 沙輪圓心z方向


        # # 共用一邊-------------------------------------------
        #
        gdt = np.array([[4.0177e+02, 1.3467e+03, -1.7707e+02]])  # 粗磨研磨點
        #
        # 建立研磨點坐標系
        gdt_o = np.array([[4.0177e+02, 1.3467e+03, -1.7707e+02]])  # 粗磨原點
        gdt_y = np.array([[4.2964e+02, 1.3476e+03, -1.7707e+02]])  # 粗磨y方向
        gdt_z = np.array([[3.9511e+02, 1.5247e+03, -1.8158e+02]])  # 沙輪圓心z方向
        gdt = np.array([[-4.6881e+02, 1.3299e+03, -1.7648e+02]])  # 細磨研磨點
        # # 建立研磨點坐標系
        # gdt_o = np.array([[-4.6881e+02, 1.3229e+03, -1.7648e+02]])   # 細磨原點
        # gdt_y = np.array([[-4.4688e+02, 1.3235e+03, -1.7648e+02]])   # 細磨y方向
        # gdt_z = np.array([[-4.7624e+02, 1.4932e+03, -1.8535e+02]])   # 沙輪圓心z方向
        # -------------------------------------------------

        yp = np.subtract(gdt_y, gdt_o)
        zp = np.subtract(gdt_z, gdt_o)

        xp = np.cross(yp, zp)
        yp = np.cross(zp, xp)

        xp = xp.flatten()
        yp = yp.flatten()
        zp = zp.flatten()
        dx = math.sqrt(Sq2(xp[0]) + Sq2(xp[1]) + Sq2(xp[2]))
        xp = xp / dx
        dy = math.sqrt(Sq2(yp[0]) + Sq2(yp[1]) + Sq2(yp[2]))
        yp = yp / dy
        dz = math.sqrt(Sq2(zp[0]) + Sq2(zp[1]) + Sq2(zp[2]))
        zp = zp / dz

        gdt_xyz = np.vstack([xp, yp, zp])
        rTT = gdt_xyz.T

        vt = rTT.T
        vtz = vt[2]
        dvtz = math.sqrt(Sq2(vt[2][0]) + Sq2(vt[2][1]) + Sq2(vt[2][2]))
        vtzp = vtz / dvtz

        # # # 砂輪消耗修正研磨點
        if file0_no < len(f0) - 30:
            depth = robot_error  # 粗磨的誤差
        else:
            depth = robot_error_fine  # 細磨的誤差

        # print("robot error = ", depth)
        # Ngdt = gdt + (depth * vtzp)    #沿著研磨點Z軸

        #
        Ngdt = [gdt[0][0], gdt[0][1] + depth, gdt[0][2]]  # 沿著機械手臂Y軸
        Ngdt = np.array(Ngdt)
        # print(Ngdt)
        #

        rTT = np.c_[rTT, Ngdt.T]
        e = [[0, 0, 0, 1]]
        rTT = np.r_[rTT, e]
        # print('rTt =\n', rTT)

        # 法蘭面相對手臂rTe ---------------------------------------------------------------------------------------------------

        rTe = np.dot(rTT, tTe)

        # rTe = rTe[0:3, 3]
        # print('rTe =\n', type(rTe))
        # print('rTe =\n', rTe)

        SaveFile('rTe_matric{}/rTe{}_{}.xyz'.format(protruison_no, protruison_no, file0_no), rTe)

        ## 計算由拉角 zyx

        R = rTe
        if R[2, 0] < 1:
            if R[2, 0] > -1:
                thetaY = math.asin(-R[2, 0])
                thetaZ = math.atan2(R[1, 0], R[0, 0])
                thetaX = math.atan2(R[2, 1], R[2, 2])
            else:
                thetaY = math.pi / 2
                thetaZ = -math.atan2(-R[1, 2], R[1, 1])
                thetaX = 0
        else:
            thetaY = -(math.pi / 2)
            thetaZ = math.atan2(-R[1, 2], R[1, 1])
            thetaX = 0

        thetaX = thetaX * 180 / math.pi
        thetaY = thetaY * 180 / math.pi
        thetaZ = thetaZ * 180 / math.pi

        Rxyz = [thetaX, thetaY, thetaZ]
        # print('Rxyz = ', Rxyz)

        xyz = np.array(rTe[0:3, 3] / 1000)

        xyzRxyz = np.append(xyz, Rxyz)
        SaveFile('xyzRxyz/xyzRxyz{}_{}.xyz'.format(protruison_no, file0_no), xyzRxyz)

        allxyzRxyz = xyzRxyz.reshape(1, -1)
        allxyzRxyz = allxyzRxyz.flatten()
        allxyzRxyz = allxyzRxyz.tolist()
        AllxyzRxyz0.append(allxyzRxyz)
        time.sleep(0.01)

    SaveFile('C:\\Users\\User\\Desktop\\hiwin_control_example\\xyzRxyz\\AllxyzRxyz{}.xyz'.format(protruison_no), AllxyzRxyz0)
    SaveFile('D:\\euler\\euler_angle\\xyzRxyz\\AllxyzRxyz{}.xyz'.format(protruison_no), AllxyzRxyz0)
