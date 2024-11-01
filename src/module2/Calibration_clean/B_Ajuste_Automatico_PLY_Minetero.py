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
from sklearn.neighbors import NearestNeighbors

def getPCD(k_list,x,frame):

    for i in k_list:
            sensor='k0'+str(i)
            pcd_name = "pcd{}".format(i)
            pcd_dict[pcd_name]=o3d.io.read_point_cloud("Points_Cloud/Frame_"+str(frame)+"/"+sensor+".ply")
            
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

                pcd_dict[pcd_name].transform(T_C_Prom)

                rms_error=0

            elif pcd_name=="pcd2":
                T_C_1_a = np.load("./Matrices_Iniciales/k02k01.npy")
                T1 = np.linalg.inv(np.load("./Matrices_Iniciales/k03k02.npy"))
                T2 = np.linalg.inv(np.load("./Matrices_Iniciales/k04k03.npy"))
                T3 = np.linalg.inv(np.load("./Matrices_Iniciales/k05k04.npy"))
                T4 = np.linalg.inv(np.load("./Matrices_Iniciales/k06k05.npy"))
                T5 = np.linalg.inv(np.load("./Matrices_Iniciales/k01k06.npy"))

                T_C_2=np.dot(np.dot(np.dot(np.dot(T5, T4), T3),T2),T1)

                T_C_Prom= (T_C_2*xp*0.5+T_C_1_a)/2
                pcd_dict[pcd_name].transform(T_C_Prom)

            
            elif pcd_name=="pcd3":
                T1_a = np.load("./Matrices_Iniciales/k02k01.npy")
                T2_a = np.load("./Matrices_Iniciales/k03k02.npy")
                T_C_1_a = np.dot(T1_a,T2_a)

                T1 = np.linalg.inv(np.load("./Matrices_Iniciales/k04k03.npy"))
                T2 = np.linalg.inv(np.load("./Matrices_Iniciales/k05k04.npy"))
                T3 = np.linalg.inv(np.load("./Matrices_Iniciales/k06k05.npy"))
                T4 = np.linalg.inv(np.load("./Matrices_Iniciales/k01k06.npy"))
                T_C_2=np.dot(np.dot(np.dot(T4, T3), T2),T1)

                T_C_Prom= (T_C_1_a+T_C_2*x)/2
            
                pcd_dict[pcd_name].transform(T_C_Prom)



            elif pcd_name=="pcd4":
                T1 = np.load("./Matrices_Iniciales/k02k01.npy")
                T2 = np.load("./Matrices_Iniciales/k03k02.npy")
                T3 = np.load("./Matrices_Iniciales/k04k03.npy")
                T_C_1=np.dot(np.dot(T1, T2), T3)

                T4 = np.linalg.inv(np.load("./Matrices_Iniciales/k05k04.npy"))
                T5 = np.linalg.inv(np.load("./Matrices_Iniciales/k06k05.npy"))
                T6 = np.linalg.inv(np.load("./Matrices_Iniciales/k01k06.npy"))
                T_C_2=np.dot(np.dot(T6, T5), T4)

                T_C_Prom= (T_C_2*x+T_C_1*x)/2

                pcd_dict[pcd_name].transform(T_C_Prom)


            elif pcd_name=="pcd5":

                T1_a = np.load("./Matrices_Iniciales/k02k01.npy")
                T2_a = np.load("./Matrices_Iniciales/k03k02.npy")
                T3_a = np.load("./Matrices_Iniciales/k04k03.npy")
                T4_a = np.load("./Matrices_Iniciales/k05k04.npy")
                T_C_1_a=np.dot(np.dot(np.dot(T1_a, T2_a), T3_a),T4_a)


                T1 = np.linalg.inv(np.load("./Matrices_Iniciales/k06k05.npy"))
                T2 = np.linalg.inv(np.load("./Matrices_Iniciales/k01k06.npy"))
                T_C_2 = np.dot(T2,T1)
                T_C_Prom= (T_C_2+T_C_1_a*x)/2

                pcd_dict[pcd_name].transform(T_C_Prom)

                
            elif pcd_name=="pcd6":
                T1_a = np.load("./Matrices_Iniciales/k02k01.npy")
                T2_a = np.load("./Matrices_Iniciales/k03k02.npy")
                T3_a = np.load("./Matrices_Iniciales/k04k03.npy")
                T4_a = np.load("./Matrices_Iniciales/k05k04.npy")
                T5_a = np.load("./Matrices_Iniciales/k06k05.npy")
                T_C_1_a=np.dot(np.dot(np.dot(np.dot(T1_a, T2_a), T3_a),T4_a),T5_a)
                
                T_C_2 = np.linalg.inv(np.load("./Matrices_Iniciales/k01k06.npy"))
                T_C_Prom= (T_C_2+T_C_1_a*xp)/2
                pcd_dict[pcd_name].transform(T_C_Prom)
            

    Overlap=( np.abs(OverLap(pcd_dict["pcd{}".format(1)],pcd_dict["pcd{}".format(5)]))+
                            np.abs(OverLap(pcd_dict["pcd{}".format(2)],pcd_dict["pcd{}".format(6)]))+
                            np.abs(OverLap(pcd_dict["pcd{}".format(3)],pcd_dict["pcd{}".format(1)]))+
                            np.abs(OverLap(pcd_dict["pcd{}".format(4)],pcd_dict["pcd{}".format(2)]))+
                            np.abs(OverLap(pcd_dict["pcd{}".format(5)],pcd_dict["pcd{}".format(3)]))+
                            np.abs(OverLap(pcd_dict["pcd{}".format(6)],pcd_dict["pcd{}".format(4)]))

                            )
            



    return Overlap

def OverLap(source,target):
    
   #distances = source.compute_point_cloud_distance(target)
   #distances_np = np.asarray(distances)
   #rms_error= np.sqrt(np.mean(distances_np ** 2))

   # Obtener los puntos de ambas nubes
   ref_pts = np.asarray(source.points)
   reg_pts = np.asarray(target.points)

   # Definir el umbral de distancia máximo
   umbral = 0.0003

   # Encontrar los puntos más cercanos en la nube de referencia para cada punto en la nube registrada
   nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(ref_pts)
   distancias, indices = nbrs.kneighbors(reg_pts)

   # Contar el número de pares coincidentes
   num_coincidentes = np.count_nonzero(distancias < umbral)

   # Calcular el porcentaje de overlap
   porcentaje_overlap = (num_coincidentes / len(reg_pts)) * 100



   return porcentaje_overlap


#Delay
dictOffsets = {
        'k01': 0,
        'k02': 0,
        'k03': 1,
        'k04': 1,
        'k05': 1,
        'k06': 1,
        'k07': 1
    }


por=np.linspace(0.1, 1, 15)
k_list=[1,2,3,4,5,6]
frame_ini=560
frame_final=560
step=10
pcd_dict = {}

for j in range(frame_ini,frame_final+step,step):
    
      x_min_1=0
      x_min_2=0
      Overlap_f=0
      Overlap_rf=0
      xp=por[0]

      
    
      for x in por:
        
        Overlap_f=getPCD(k_list,x,j)
        x_min_1=Overlap_f 
        #delta_x=np.abs(x_min_r-x_min_1)

        if x_min_1 >Overlap_rf:
            Overlap_rf=x_min_1
            xp=x
        

        print(x,x_min_1)

      print("resultado final:")
      print(xp,Overlap_rf)
        
