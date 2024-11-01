import sys
import cv2
import open3d as o3d
import json
import threading
import os
import numpy as np
from queue import Queue
import copy
import time
import multiprocessing as mp
from shutil import rmtree

def getMa(k_list,xp):

    pcdto=o3d.geometry.PointCloud()

    carpeta = './Final_Ma'

    # Verificar si la carpeta ya existe
    if not os.path.exists(carpeta):
        # Si no existe, crear la carpeta
        os.makedirs(carpeta)

    for i in k_list:
            pcd_name = "pcd{}".format(i)
            
            if pcd_name=="pcd1":
                T1_a = np.load("./Matrices_Iniciales/k02k01.npy")
                T2_a = np.load("./Matrices_Iniciales/k03k02.npy")
                T3_a = np.load("./Matrices_Iniciales/k04k03.npy")
                T4_a = np.load("./Matrices_Iniciales/k05k04.npy")
                T5_a = np.load("./Matrices_Iniciales/k06k05.npy")
                T6_a = np.load("./Matrices_Iniciales/k01k06.npy")
                
                T_C_1_a = np.dot(np.dot(np.dot(np.dot(np.dot(T1_a, T2_a), T3_a),T4_a),T5_a),T6_a)
                
                T1 = np.linalg.inv(np.load("./Matrices_Iniciales/k02k01.npy"))
                T2 = np.linalg.inv(np.load("./Matrices_Iniciales/k03k02.npy"))
                T3 = np.linalg.inv(np.load("./Matrices_Iniciales/k04k03.npy"))
                T4 = np.linalg.inv(np.load("./Matrices_Iniciales/k05k04.npy"))
                T5 = np.linalg.inv(np.load("./Matrices_Iniciales/k06k05.npy"))
                T6 = np.linalg.inv(np.load("./Matrices_Iniciales/k01k06.npy"))

                T_C_2 = np.dot(np.dot(np.dot(np.dot(np.dot(T6, T5), T4),T3),T2),T1)

                T_C_Prom= (T_C_2+T_C_1_a*xp*2)/2
                np.save('./Final_Ma/pcd1.npy', T_C_Prom)


            elif pcd_name=="pcd2":
                T_C_1_a = np.load("./Matrices_Iniciales/k02k01.npy")
                T1 = np.linalg.inv(np.load("./Matrices_Iniciales/k03k02.npy"))
                T2 = np.linalg.inv(np.load("./Matrices_Iniciales/k04k03.npy"))
                T3 = np.linalg.inv(np.load("./Matrices_Iniciales/k05k04.npy"))
                T4 = np.linalg.inv(np.load("./Matrices_Iniciales/k06k05.npy"))
                T5 = np.linalg.inv(np.load("./Matrices_Iniciales/k01k06.npy"))

                T_C_2=np.dot(np.dot(np.dot(np.dot(T5, T4), T3),T2),T1)

                T_C_Prom= (T_C_2*xp*0.5+T_C_1_a)/2
                np.save('./Final_Ma/pcd2.npy', T_C_Prom)
            
            elif pcd_name=="pcd3":
                T1_a = np.load("./Matrices_Iniciales/k02k01.npy")
                T2_a = np.load("./Matrices_Iniciales/k03k02.npy")
                T_C_1_a = np.dot(T1_a,T2_a)

                T1 = np.linalg.inv(np.load("./Matrices_Iniciales/k04k03.npy"))
                T2 = np.linalg.inv(np.load("./Matrices_Iniciales/k05k04.npy"))
                T3 = np.linalg.inv(np.load("./Matrices_Iniciales/k06k05.npy"))
                T4 = np.linalg.inv(np.load("./Matrices_Iniciales/k01k06.npy"))
                T_C_2=np.dot(np.dot(np.dot(T4, T3), T2),T1)

                T_C_Prom= (T_C_1_a+T_C_2*xp)/2
                np.save('./Final_Ma/pcd3.npy', T_C_Prom)
            
            elif pcd_name=="pcd4":
                T1 = np.load("./Matrices_Iniciales/k02k01.npy")
                T2 = np.load("./Matrices_Iniciales/k03k02.npy")
                T3 = np.load("./Matrices_Iniciales/k04k03.npy")
                T_C_1=np.dot(np.dot(T1, T2), T3)

                T4 = np.linalg.inv(np.load("./Matrices_Iniciales/k05k04.npy"))
                T5 = np.linalg.inv(np.load("./Matrices_Iniciales/k06k05.npy"))
                T6 = np.linalg.inv(np.load("./Matrices_Iniciales/k01k06.npy"))
                T_C_2=np.dot(np.dot(T6, T5), T4)

                T_C_Prom= (T_C_2*xp+T_C_1*xp)/2
                np.save('./Final_Ma/pcd4.npy', T_C_Prom)


            elif pcd_name=="pcd5":

                T1_a = np.load("./Matrices_Iniciales/k02k01.npy")
                T2_a = np.load("./Matrices_Iniciales/k03k02.npy")
                T3_a = np.load("./Matrices_Iniciales/k04k03.npy")
                T4_a = np.load("./Matrices_Iniciales/k05k04.npy")
                T_C_1_a=np.dot(np.dot(np.dot(T1_a, T2_a), T3_a),T4_a)


                T1 = np.linalg.inv(np.load("./Matrices_Iniciales/k06k05.npy"))
                T2 = np.linalg.inv(np.load("./Matrices_Iniciales/k01k06.npy"))
                T_C_2 = np.dot(T2,T1)
                T_C_Prom= (T_C_2+T_C_1_a*xp)/2
                np.save('./Final_Ma/pcd5.npy', T_C_Prom)

                
            elif pcd_name=="pcd6":
                T1_a = np.load("./Matrices_Iniciales/k02k01.npy")
                T2_a = np.load("./Matrices_Iniciales/k03k02.npy")
                T3_a = np.load("./Matrices_Iniciales/k04k03.npy")
                T4_a = np.load("./Matrices_Iniciales/k05k04.npy")
                T5_a = np.load("./Matrices_Iniciales/k06k05.npy")
                T_C_1_a=np.dot(np.dot(np.dot(np.dot(T1_a, T2_a), T3_a),T4_a),T5_a)
                
                T_C_2 = np.linalg.inv(np.load("./Matrices_Iniciales/k01k06.npy"))
                T_C_Prom= (T_C_2+T_C_1_a*xp)/2
                np.save('./Final_Ma/pcd6.npy', T_C_Prom)
            
           

    return pcdto


def getPCDs(frame,k_list):
    
    sensor='k0'+str(k_list)
    pcd_name = "pcd{}".format(k_list)
    pcd_dict[pcd_name]=o3d.io.read_point_cloud("Points_Cloud/Frame_"+str(frame)+"/"+sensor+".ply")
   
    T_C_Prom = np.load("./Final_Ma/"+pcd_name+".npy")
    pcd_dict[pcd_name].transform(T_C_Prom)
       
    pcdto=pcd_dict[pcd_name]

    return pcdto

def getIntrisic(intrinsicpath):
    intrinsic = None
    with open(intrinsicpath) as f:
        data = json.load(f)
        width = data['width']
        height = data['height']
        intrinsic_matrix = data['intrinsic_matrix']
        fx, fy, cx, cy = intrinsic_matrix[0], intrinsic_matrix[4], intrinsic_matrix[6], intrinsic_matrix[7]
        intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    return intrinsic

carpeta1 = './Points_Cloud_cal_Ajustado'

# Verificar si la carpeta ya existe
if not os.path.exists(carpeta1):
    # Si no existe, crear la carpeta
    os.makedirs(carpeta1)



k_list=[1,2,3,4,5,6]#segun el numero de camaras
frame_ini=560
frame_final=560
step=20
xp=0.6142857142857143

get_ma=getMa(k_list,xp) #Genera las matrices de ajustadas
pcdto=o3d.geometry.PointCloud()
pcd_dict = {}


for j in range(frame_ini,frame_final+step,step):

    print("Frame:", j)

    carpeta2 = "Points_Cloud_cal_Ajustado/Frame_"+str(j)

    # Verificar si la carpeta ya existe
    if not os.path.exists(carpeta2):
        # Si no existe, crear la carpeta
        os.makedirs(carpeta2)

    for i in k_list:
        pcdto=getPCDs(frame_ini,i)

        o3d.io.write_point_cloud("Points_Cloud_cal_Ajustado/Frame_"+str(j)+"/k0"+str(i)+".ply", pcdto)
        print("k0"+str(i)+" of "+str(len(k_list)))