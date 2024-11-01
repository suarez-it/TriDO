
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
from sklearn.neighbors import NearestNeighbors

def getPCD(frame,k_list):

    pcdto=o3d.geometry.PointCloud()
    pcd_dict = {}

    carpeta = './Final_Ma'

    # Verificar si la carpeta ya existe
    if not os.path.exists(carpeta):
        # Si no existe, crear la carpeta
        os.makedirs(carpeta)

    for i in k_list:
            sensor='k0'+str(i)
            pcd_name = "pcd{}".format(i)
            pcd_dict[pcd_name]=o3d.io.read_point_cloud("Points_Cloud/Frame_"+str(frame)+"/"+sensor+".ply")
            
            if pcd_name=="pcd1":
                
                T1 = np.load("./Final_Ma/pcd1.npy")
                pcd_dict[pcd_name].transform(T1)


            elif pcd_name=="pcd2":
                T2 = np.load("./Final_Ma/pcd2.npy")
                pcd_dict[pcd_name].transform(T2)

            
            elif pcd_name=="pcd3":
                T3 = np.load("./Final_Ma/pcd3.npy")
                pcd_dict[pcd_name].transform(T3)



            elif pcd_name=="pcd4":
                T4 = np.load("./Final_Ma/pcd4.npy")
                pcd_dict[pcd_name].transform(T4)


            elif pcd_name=="pcd5":

                T5 = np.load("./Final_Ma/pcd5.npy")
                pcd_dict[pcd_name].transform(T5)

                
            elif pcd_name=="pcd6":
                T6 = np.load("./Final_Ma/pcd6.npy")
                pcd_dict[pcd_name].transform(T6)
                    
            pcdto=pcdto+pcd_dict[pcd_name]

    return pcdto

def getOverlap(source,target,threshold):
    
   # Obtener los puntos de ambas nubes
   ref_pts = np.asarray(source.points)
   reg_pts = np.asarray(target.points)

   # Encontrar los puntos más cercanos en la nube de referencia para cada punto en la nube registrada
   nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(ref_pts)
   distancias, indices = nbrs.kneighbors(reg_pts)

   # Contar el número de pares coincidentes
   num_coincidentes = np.count_nonzero(distancias < threshold)

   # Calcular el porcentaje de overlap
   overlap = (num_coincidentes / len(reg_pts)) * 100


   return overlap


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


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    #source_temp.paint_uniform_color([1, 0.706, 0])
    #target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def pick_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) Afther picking points, press q for close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()



def manual_calibration(device_index_source, device_index_target,frame_ini, frame_fin,step):
    #try:
        print("Demo for manual ICP")

        if device_index_source!="k07":
            source = o3d.io.read_point_cloud("Points_Cloud/Frame_"+str(frame_ini)+"/"+device_index_source+".ply")
            target = o3d.io.read_point_cloud("Points_Cloud/Frame_"+str(frame_ini)+"/"+device_index_target+".ply")

        else:
            source = o3d.io.read_point_cloud("Points_Cloud/Frame_"+str(frame_ini)+"/"+device_index_source+".ply")
            target=getPCD(frame_ini,[1,2,3,4,5,6])
        
        
        print("Visualization of two point clouds before manual alignment")
        draw_registration_result(source, target, np.asarray([[1, 0, 0, 0],[0,1,0,0.5],[0,0,1,0],[0,0,0,1]]))

        # pick points from two point clouds and builds correspondences
        picked_id_source = pick_points(source)
        picked_id_target = pick_points(target)
        assert (len(picked_id_source) >= 3 and len(picked_id_target) >= 3)
        assert (len(picked_id_source) == len(picked_id_target))
        corr = np.zeros((len(picked_id_source), 2))
        corr[:, 0] = picked_id_source
        corr[:, 1] = picked_id_target

        print(picked_id_source)
        print(picked_id_target)

        # estimate rough transformation using correspondences
        print("Compute a rough transform using the correspondences given by user")
        p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        trans_init = p2p.compute_transformation(source, target,
                                                    o3d.utility.Vector2iVector(corr))

        # point-to-point ICP for refinement
        print("Perform point-to-point ICP refinement")
        threshold = 0.0003

        reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200,
                                                                relative_fitness=1e-6, 
                                                                relative_rmse=1e-6))
                
        trans_init = reg_p2p.transformation

            
        # Transforma los puntos de origen usando la transformación resultante
        source_transformed = source.transform(trans_init)

        #Calculo de Overlap
        overlap_1= getOverlap(source_transformed,target,threshold)
            
        trans_init_f=trans_init


        for i in range(frame_ini,frame_fin,step):

          source = o3d.io.read_point_cloud("Points_Cloud/Frame_"+str(i)+"/"+device_index_source+".ply")
          target = o3d.io.read_point_cloud("Points_Cloud/Frame_"+str(i)+"/"+device_index_target+".ply")
            
          reg_p3p = o3d.pipelines.registration.registration_icp(
          source, target, threshold,  trans_init_f,
          o3d.pipelines.registration.TransformationEstimationPointToPoint(),
          o3d.pipelines.registration.ICPConvergenceCriteria( max_iteration=200,
                                                            relative_fitness=1e-6, 
                                                            relative_rmse=1e-6))
          trans_init = reg_p3p.transformation
          # Transforma los puntos de origen usando la transformación resultante
          source_transformed_2 = source.transform(trans_init)

          overlap_2= getOverlap(source_transformed_2,target,threshold)

          print("%_Overlap: ",overlap_1, " ---- Frame: ", i)

          if overlap_2>overlap_1:
              overlap_1=overlap_2
              trans_init_f=trans_init

        print("Final_Matriz:",trans_init_f)

        if device_index_source=="k07":
            np.save('./Final_Ma/pcd7.npy',trans_init_f)

        result = {'Result': True, 'Transformation': trans_init_f}
        np.save(device_index_source+device_index_target+'.npy', trans_init_f)
    #except:
        #print('Could not calibrate')
        #result = {'Result': False, 'Transformation': None}
        return result


#Definir el fram de arranque y el frame final
frame_ini=420
frame_final=820
step=20 #cada cuanto voy a generar un frame
k_list=[1,7] #Ajuste camara superior - Ejecutar despues del ajustar la camaras laterales

for w in range(0,len(k_list)):
    
    if w<len(k_list)-1:
       
      sensor_1='k0'+str(k_list[w])
      sensor_2='k0'+str(k_list[w+1])
      print("Ajuste entre camaras: "+ sensor_1+"--"+sensor_2)
      result=manual_calibration(sensor_2,sensor_1,frame_ini,frame_final,step)
