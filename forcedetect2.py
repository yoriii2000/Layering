import serial
import time
import binascii
import re
import numpy as np
import matplotlib.pyplot as plt
import os
import math

import single_pro_euler

def Sq2(value):
    # Code untuk Kuadrat bilangan
    return value*value

if __name__ == '__main__':

    # 通訊用資txt
    file_path = "connect_sensor.txt"
    file_exist_status = os.path.isfile(file_path)
    print("file_exist_status")
    print(file_exist_status)
    reviseT_path = "revise_Tpoint.txt"

    run = -1
    resultfile_no = 0
    averageline_time_file = open("force data/average1line_times.txt", 'w').close()
    total_F = open("force data/total_F.txt", 'w').close()
    touch_or_not = 0
    while 1:
        if os.path.exists(file_path):
            averageline_time_file = open("force data/average1line_times.txt", 'a+')
            total_F = open("force data/total_F.txt", 'a+')

            # ti = np.genfromtxt(averageline_time_file, dtype=None, comments='#', delimiter=' ')

            # 成功讀到force sensor回傳給C+開始移動
            return_robot_file = "return_robot.txt"

            # value = np.genfromtxt(file_path, dtype=None, comments='#', delimiter=' ')

            # path = 'text.txt'
            f = open(file_path, 'r')
            time.sleep(0.1)
            value = int(f.read())
            resultfile_no = resultfile_no + 1
            if value == 1:
                run = run + 1
                print("run = ", run)
                # for line_no in range(3):
                ser = serial.Serial()
                ser.port = "COM6"
                ser.baudrate = 115200
                ser.bytesize = serial.EIGHTBITS  # number of bits per bytes
                ser.parity = serial.PARITY_NONE  # set parity check
                ser.stopbits = serial.STOPBITS_ONE  # number of stop bits
                ser.timeout = 0.1  # non-block read 0.5s
                ser.writeTimeout = 0.1  # timeout for write 0.5s
                ser.xonxoff = False  # disable software flow control
                ser.rtscts = False  # disable hardware (RTS/CTS) flow control
                ser.dsrdtr = False  # disable hardware (DSR/DTR) flow control

                times = np.array([])
                datafx = np.array([])
                datafy = np.array([])
                datafz = np.array([])
                dataF = np.array([])
                begin_force = 0
                loop_no = np.array([])
                F_loop_no = np.array([])
                resultdata = np.array([])
                re_time = np.array([])
                aftertouch_times = np.array([])
                aftertouch_dataF = np.array([])
                # starttime = time.time()

                plt.figure(figsize=(8, 4), dpi=95)
                plt.ion()
                plt.suptitle('Force Data', fontsize=20)
                plt.show()

                try:
                    ser.open()

                except Exception as ex:
                    print("open serial port error " + str(ex))
                    exit()

                cmd = b'c\r'
                # print('cmd', cmd)
                # input('stop')
                # for line_no in range(3):
                # if run % 3 == 0:
                #     print('file number = ', int(run / 3), int(1))
                #     text_file = open("force data/datadetect{}_{}.txt".format(int(run / 3), int(1)), 'w')
                #     force_file = open("force data/F_line{}_{}.txt".format(int(run / 3), int(1)), 'w')
                # elif round(run / 3) == int(run / 3):
                #     print('file number = ', int(run / 3), int(2))
                #     text_file = open("force data/datadetect{}_{}.txt".format(int(run / 3), int(2)), 'w')
                #     force_file = open("force data/F_line{}_{}.txt".format(int(run / 3), int(2)), 'w')
                # elif round(run / 3) == (int(run / 3) + 1):
                #     print('file number = ', int(run / 3), int(3))
                #     text_file = open("force data/datadetect{}_{}.txt".format(int(run / 3), int(3)), 'w')
                #     force_file = open("force data/F_line{}_{}.txt".format(int(run / 3), int(3)), 'w')

                text_file = open("force data/datadetect{}.txt".format(run), 'w')
                if run != 0:
                    force_file = open("force data/F_line_part2.txt".format(run), 'a')

                else:
                    force_file = open("force data/F_line_part1.txt".format(run), 'w')




                if ser.isOpen():
                    # send start command "c\r"
                    #
                    # while (1):
                    #
                    #     ser.write(cmd)
                    #     print("write : %s" % cmd)
                    #
                    #     response = ser.read(90)
                    #     print("fread : %s" % response)
                    #     if len(response) > 50:
                    #         break
                    #
                    # time.sleep(0.5)  # wait 0.5s
                    # # read force command
                    starttime = time.time()
                    print("start")
                    #
                    whilerun = 0
                    while 1:
                        try:
                            # ser.flushInput()  # flush input buffer
                            # ser.flushOutput()  # flush output buffer
                            print("\nclear last")
                            print('sensor sit', ser.isOpen())

                            # read 80 byte data
                            # response = ser.read(100)
                            response = ser.readline()
                            # print("read : %s" % response)

                            if len(response) > 50:
                                whilerun = whilerun + 1
                                # print('whilerun = ', whilerun)
                                print("read : %s" % response)
                                endtime = time.time()

                                newres = response.decode('UTF-8', 'strict')
                                b = re.split(r'\s+', newres)

                                for i in range(b.count('')):
                                    b.remove('')

                                force = b[::2]
                                sensordata = force[:3]

                                fx = float(sensordata[0])
                                fy = float(sensordata[1])
                                fz = float(sensordata[2])

                                # 力感測總和力
                                F = (fx ** 2 + fy ** 2 + fz ** 2) ** 0.5
                                F = (round(F*1000))/1000

                                forcedata = [fx, fy, fz]
                                # print('forcedata = ', forcedata)
                                # print('F = ', F)
                                processtime = endtime - starttime
                                print("耗時: {:.2f}秒".format(processtime))

                                X = round(processtime, 2)
                                Yx = fx
                                Yy = fy
                                Yz = fz
                                YF = F

                                strx = str(X)
                                strF = str(F)
                                newres = strx + ' s\t' + newres            # sensor接收到的六個值
                                Fxyz = strx + ' s\t' + strF + ' N\t\n'     # 總合力
                                total_Fxyz = strx + '\t' + strF + '\t\n'

                                text_file.write(newres)
                                force_file.write(Fxyz)
                                total_F.write(total_Fxyz)

                                # print('text_file = ', type(newres))
                                if response == "\n":
                                    text_file.seek()
                                    force_file.seek()
                                    text_file.truncate()
                                    force_file.truncate()
                                text_file.flush()
                                force_file.flush()

                                resultdata = np.vstack([X, fx, fy, fz])
                                resultdata = resultdata.T

                                times = np.append(times, X)
                                dataF = np.append(dataF, YF)

                                # 計算力質變化
                                # diff_F = float()
                                # if len(dataF) > 2:
                                #     diff_F = abs(dataF[len(dataF) - 1] - dataF[len(dataF) - 2])
                                # print("whilerun = ", whilerun)
                                # print('diff_F = ', (round(diff_F*1000))/1000)

                                if len(dataF) == 10:
                                    begin_force = float(np.sum(dataF) / len(dataF))
                                print("begin_force = ", begin_force)

                                plt.clf()

                                if run == 0:
                                    # 利用 力值變化 判斷碰到研磨點
                                    # if 3 > diff_F > 2:
                                    #     loop_no = np.append(loop_no, whilerun)
                                    # print("loop_no = ", loop_no)
                                    New_detect_F = abs(dataF[len(dataF) - 1])
                                    print("NEW DETECT Force = ", New_detect_F)
                                    Real_Force = abs(New_detect_F - begin_force)
                                    print("REAL TIME Force = ", Real_Force)

                                    # 利用 力值 判斷碰到研磨點
                                    if Real_Force > 8:
                                        loop_no = np.append(loop_no, whilerun)
                                    print("loop_no len= ", len(loop_no))

                                    # print("loop_no = ", loop_no)

                                    print("touch_or_not = ", touch_or_not)
                                    print("loop_no[len(loop_no) - 1]", loop_no[len(loop_no) - 1])
                                    print("loop_no[len(loop_no) - 3]", loop_no[len(loop_no) - 3])
                                    print("loop_no[len(loop_no) - 2]", loop_no[len(loop_no) - 2])

                                    noise = (loop_no[len(loop_no) - 1] + loop_no[len(loop_no) - 3]) / 2
                                    print("mid noise = ", noise)
                                    print("confirm no = ", loop_no[len(loop_no) - 2])
                                    if len(loop_no) > 7 and (noise + 2) >= (loop_no[len(loop_no) - 2]) >= (noise - 2):
                                        touch_or_not = 1
                                        F_loop_no = np.append(F_loop_no, whilerun)
                                        print("F_loop_no = ", F_loop_no)

                                        print("touch_or_not = ", touch_or_not)
                                    else:
                                        print("no touch")


                                    # if touch_or_not == 1:
                                    if whilerun == F_loop_no[0]:
                                        # 確定接觸到研磨點，回傳1 1
                                        ru = open(return_robot_file, "w")
                                        ru.write(str(1))
                                        ru.close()
                                        print('return robot')
                                        time.sleep(1.5)

                                        # 接收研磨點修正距離
                                        revise = open(reviseT_path, 'r')
                                        line = revise.readlines()
                                        revise_txt = []
                                        for x in range(0, len(line)):
                                            revise_txt = line[x].split(' ', 1)  # [x y z] from File
                                        print('研磨點誤差', revise_txt)

                                        protruison_no = int(revise_txt[0])
                                        revise_dis = float(revise_txt[1])

                                        time.sleep(0.1)
                                        print("protrusion = ", protruison_no)
                                        print("revise distance = ", revise_dis)

                                        T, newT = single_pro_euler.euler(protruison_no, revise_dis)
                                        time.sleep(0.1)

                                        re_finish = open(return_robot_file, "w")
                                        re_finish.write(str(2))
                                        re_finish.close()

                                        print("original grinding point = ", T)
                                        print("new grinding point = ", newT)
                                        distance = math.sqrt(Sq2(newT[0] - T[0][0]) + Sq2(newT[1] - T[0][1]) + Sq2(newT[2] - T[0][2]))
                                        print("real revise distance = ", distance)

                                        touch_time = time.time()
                                        recompute_time = touch_time - starttime
                                        re_time = round(recompute_time, 2)

                                        print("開始到碰到研磨點的時間", re_time)

                                    elif whilerun > F_loop_no[0]:
                                        print("開始研磨")
                                        if X > re_time:
                                            aftertouch_times = np.append(aftertouch_times, X)
                                            aftertouch_dataF = np.append(aftertouch_dataF, YF)

                                        plt.subplot()
                                        plt.xlabel("time(s)")
                                        plt.ylabel("F(N)")
                                        # print(aftertouch_times, aftertouch_dataF)
                                        plt.grid()
                                        plt.scatter(aftertouch_times, aftertouch_dataF, color='tab:red')
                                        plt.plot(aftertouch_times, aftertouch_dataF, color='red')

                                    plt.pause(0.001)
                                    # if run % 3 == 0:
                                    #     plt.savefig("force data/plot{}_{}".format(int(run / 3), int(1)))
                                    #
                                    # elif round(run / 3) == int(run / 3):
                                    #     plt.savefig("force data/plot{}_{}".format(int(run / 3), int(2)))
                                    #
                                    # elif round(run / 3) == (int(run / 3) + 1):
                                    #     plt.savefig("force data/plot{}_{}".format(int(run / 3), int(3)))

                                    plt.savefig("force data/plot{}".format(run))
                                else:
                                    print('run', run)
                                    ru = open(return_robot_file, "w")
                                    ru.write(str(1))
                                    ru.close()
                                    print('return robot')

                                    plt.subplot()
                                    plt.xlabel("time(s)")
                                    plt.ylabel("Fx(N)")
                                    plt.grid()
                                    # print(times, dataF)
                                    plt.scatter(times, dataF, color='tab:red')
                                    plt.plot(times, dataF, color='red')
                                    plt.savefig("force data/plot{}".format(run))


                            backfile = open(file_path, 'r')
                            backvalue = int(backfile.read())
                            print()
                            if backvalue == 2:
                                oneline_endtime = time.time()

                                # ---------將原本的檔案覆蓋掉裡面的數值----------
                                f = open(file_path, "w")
                                f.write(str(0))
                                f.close()

                                # 離開force sensor回傳0
                                ru = open(return_robot_file, "w")
                                ru.write(str(0))
                                ru.close()

                                duringtime = oneline_endtime - starttime
                                print("oneline耗時: {:.2f}秒".format(duringtime))
                                duringtime = round(duringtime, 2)
                                strduringtime = str(duringtime)
                                oneline_time = strduringtime + ' s\t\n'
                                averageline_time_file.write(oneline_time)
                                plt.close(fig=None)
                                break
                                # ------------------------------------
                                # sys.exit()

                        except Exception as e1:
                            print("communicating error " + str(e1))

                    ser.close()

                else:
                    print("open serial port error")
            else:
                print("wait")
        else:
            print("檔案路徑不存在。")






