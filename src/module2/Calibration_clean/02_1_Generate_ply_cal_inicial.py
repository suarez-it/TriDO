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

def getPCDs(frame,k_list):
    
    sensor='k0'+str(k_list)
    pcd_name = "pcd{}".format(k_list)
    pcd_dict[pcd_name]=o3d.io.read_point_cloud("Points_Cloud/Frame_"+str(frame)+"/"+sensor+".ply")

    if pcd_name=="pcd2":
        T = np.load("./Matrices_iniciales/k02k01.npy")
        pcd_dict[pcd_name].transform(T)
    
    elif pcd_name=="pcd3":
        T1= np.load("./Matrices_Iniciales/k02k01.npy")
        T2= np.load("./Matrices_Iniciales/k03k02.npy")
        T = np.dot(T1,T2)
        pcd_dict[pcd_name].transform(T)
    
    elif pcd_name=="pcd4":
        T1 = np.load("./Matrices_Iniciales/k02k01.npy")
        T2 = np.load("./Matrices_Iniciales/k03k02.npy")
        T3 = np.load("./Matrices_Iniciales/k04k03.npy")
        T=np.dot(np.dot(T1, T2), T3)
        pcd_dict[pcd_name].transform(T)
    
    elif pcd_name=="pcd5":
        T1 = np.load("./Matrices_Iniciales/k02k01.npy")
        T2 = np.load("./Matrices_Iniciales/k03k02.npy")
        T3 = np.load("./Matrices_Iniciales/k04k03.npy")
        T4 = np.load("./Matrices_Iniciales/k05k04.npy")
        T  = np.dot(np.dot(np.dot(T1, T2), T3),T4)
        pcd_dict[pcd_name].transform(T)

    elif pcd_name=="pcd6":
        T1 = np.load("./Matrices_Iniciales/k02k01.npy")
        T2 = np.load("./Matrices_Iniciales/k03k02.npy")
        T3 = np.load("./Matrices_Iniciales/k04k03.npy")
        T4 = np.load("./Matrices_Iniciales/k05k04.npy")
        T5 = np.load("./Matrices_Iniciales/k06k05.npy")
        T  = np.dot(np.dot(np.dot(np.dot(T1, T2), T3),T4),T5)
        
        pcd_dict[pcd_name].transform(T)

    elif pcd_name=="pcd7":
        T = np.load("./Matrices_iniciales/k01k07.npy")
        pcd_dict[pcd_name].transform(T)

    return pcd_dict[pcd_name]

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

carpeta1 = './Points_Cloud_Cal_inicial'

# Verificar si la carpeta ya existe
if not os.path.exists(carpeta1):
    # Si no existe, crear la carpeta
    os.makedirs(carpeta1)

#Parametros de calibracion
k_list=[1,2,3,4,5,6]#segun el numero de camaras
frame_ini=560
frame_final=560
step=10


pcdto=o3d.geometry.PointCloud()
pcd_dict = {}


for j in range(frame_ini,frame_final+step,step):

    print("Frame:", j)

    carpeta = "Points_Cloud_Cal_inicial/Frame_"+str(j)

    # Verificar si la carpeta ya existe
    if not os.path.exists(carpeta):
        # Si no existe, crear la carpeta
        os.makedirs(carpeta)

    for i in k_list:
        pcdto=getPCDs(j,i)

        o3d.io.write_point_cloud("Points_Cloud_Cal_inicial/Frame_"+str(j)+"/k0"+str(i)+".ply", pcdto)
        print("k0"+str(i)+" of "+str(len(k_list)))