*研磨注意事項*
*1. force sensor要歸0 (bias on)
2. RTe&single euler 預留誤差要一致! 超重要!!!!
3. forcedetect3.py要開啟
4. 研磨完記得儲存力資料 他媽的
5.revise_Tpoint.txt 也要記得存!*


trajectory.py

trajectory1.py  #手動選擇凸點邊緣

trajectory2.py  #自動選擇凸點邊緣

ReduceTrajectory.py + Tcoorciate2.py + RTe_to_ruler.py   #防止過磨(看起來效果偏低)

RTe_to_ruler.py  #粗磨+細磨

手臂誤差 = "-" 10


RTe_to_ruler2.py  #手臂誤差分開輸入 (垃圾)

![image](https://github.com/yoriii2000/fuckuhsuzting/assets/111038997/f6ea2b25-d827-4232-91d0-006fa44feb78)


Force sensor 

![image](https://github.com/yoriii2000/fuckuhsuzting/assets/111038997/23880a50-f6be-44f4-ba98-27e4476bc0c9)



inc_trajectory.py  #研磨完確認研磨程度，按不同程度要求增加軌跡


forcedetect4.py # 學長程式重新理解版，更漂亮ㄌ

2024/01/11
1. 為了研磨磨耗塊
更改了機械手臂inter position
*但要取決於凸點是從左到右研磨還是右到左*
*防止第四軸轉180度*
2. estNormal.py 更改KDTree的 Radius，凸點越大 Radius越大(小:20 中:25 大:30)
3. 大的磨塊一層最好0.1mm就好 中的最多0.2mm

2024/01/19
1. 擬合磨塊2_1曲面(未完成)

2024/03/12
實驗2實驗經驗又增加ㄌ的部分
1. 從大研磨塊開始磨
2. 點到點要移速一致  
3. 重新計算軌跡時將手臂拉出
4. 校正完砂輪ㄐ要重新取研磨點(不要懶啦)
5. 研磨時ㄉ手臂姿態!每顆都要一樣!

2024/3/14
1. 粗磨要逆磨
2. 細磨要順磨

2024/03/15
1. 手臂姿態導致Fx FY RX RZ有正負的差別
2. 力感測器接收到的資料有空窗期，為計算新軌跡ㄉ時間

