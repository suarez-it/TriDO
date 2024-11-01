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

def getPCD(frame_n,k_list,filtro,T1,T2,T3,T4,T5,T6):
    
    pcdto=o3d.geometry.PointCloud()
    pcd_dict = {}

    #Delay
    dictOffsets = {
        'k01': 1,
        'k02': 2,
        'k03': 2,
        'k04': 2,
        'k05': 2,
        'k06': 2,
        'k07': 0
    }
    

    for i in k_list:
            sensor='k0'+str(i)
            frame = str(frame_n+dictOffsets[sensor]).zfill(5)
            pcd_name = "pcd{}".format(i)
            pcd_dict[pcd_name]=create3DFromImages_Filter_Border(sensor,frame,filtro)
            

            if pcd_name=="pcd1":
                
                
                pcd_dict[pcd_name].transform(T1)


            elif pcd_name=="pcd2":
               
                pcd_dict[pcd_name].transform(T2)

            
            elif pcd_name=="pcd3":
                
                pcd_dict[pcd_name].transform(T3)



            elif pcd_name=="pcd4":
                
                pcd_dict[pcd_name].transform(T4)


            elif pcd_name=="pcd5":

                
                pcd_dict[pcd_name].transform(T5)

                
            elif pcd_name=="pcd6":
                
                pcd_dict[pcd_name].transform(T6)
            
            elif pcd_name=="pcd7":
                
                pcd_dict[pcd_name].transform(T7)
                    
            pcdto=pcdto+pcd_dict[pcd_name]
    
    # Definir la matriz de transformación de rotación sobre el eje y
    theta = np.pi / 10  # Ángulo de rotación en radianes (en este caso, 45 grados)
    Ry = np.array([[np.cos(theta), 0, np.sin(theta)],
                [0, 1, 0],
                [-np.sin(theta), 0, np.cos(theta)]])

    # Aplicar la transformación a la nube de puntos
    pcdto.rotate(Ry)


    return pcdto


def getIntrisic(intrinsicpath):
    intrinsic = None
    with open(intrinsicpath) as f:
        scale_percent=100
        data = json.load(f)
        width = int(data['width']*scale_percent / 100)
        height = int(data['height']* scale_percent / 100)
        intrinsic_matrix = data['intrinsic_matrix']
        fx, fy, cx, cy = intrinsic_matrix[0]*scale_percent / 100, intrinsic_matrix[4]*scale_percent / 100, intrinsic_matrix[6]*scale_percent / 100, intrinsic_matrix[7]*scale_percent / 100
        intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    return intrinsic


def create3DFromImages_Filter_Border(sensor,frame,filtro):
    scale_percent= 100

    im_cv = cv2.imread(sensor+'/color/'+frame+'.jpg')
    color_image  = cv2.cvtColor(im_cv, cv2.COLOR_BGR2RGB)
    
    depth_gray=cv2.imread(sensor+'/depth/'+frame+'.png',cv2.IMREAD_GRAYSCALE)
    depth_image = cv2.imread(sensor+'/depth/'+frame+'.png',-1)
    
    # Redimensionar la imagen
    width = int(color_image.shape[1] * scale_percent / 100)
    height = int(color_image.shape[0] * scale_percent / 100)
    dim = (width, height)
    color_image = cv2.resize(color_image, dim, interpolation=cv2.INTER_AREA)
    depth_gray= cv2.resize(depth_gray, dim, interpolation=cv2.INTER_AREA)
    depth_image= cv2.resize(depth_image, dim, interpolation=cv2.INTER_AREA)
    
   
    
    # Apply erosion to the depth image
    kernel = np.array([[0, 1, 0],
                [1, 1, 1],
                 [0, 1, 0]], dtype=np.uint8)
    
    if filtro==1: 
        mask_1= depth_gray
        dilated_image = cv2.dilate(mask_1, kernel, iterations=2)
        eroded_image  = cv2.erode(dilated_image, kernel, iterations=10)
        color_image_result = cv2.bitwise_and(color_image ,color_image , mask=eroded_image)
        depth_image_result = cv2.bitwise_and(depth_image ,depth_image, mask=eroded_image)
    
    elif filtro==2: 
        mask_1= cv2.inRange(depth_gray ,1,20)
        eroded_image  = cv2.erode(mask_1, kernel, iterations=8)
        dilated_image = cv2.dilate(eroded_image, kernel, iterations=2)
        color_image_result = cv2.bitwise_and(color_image ,color_image , mask=dilated_image)
        depth_image_result = cv2.bitwise_and(depth_image ,depth_image, mask=dilated_image)
    
    elif filtro==3:
        mask_1= cv2.inRange(depth_gray ,1,20)
        eroded_image_a  = cv2.erode(mask_1, kernel, iterations=12)
        dilated_image = cv2.dilate(eroded_image_a, kernel, iterations=10)
        eroded_image  = cv2.erode(dilated_image, kernel, iterations=6)

        mask_inverted = cv2.bitwise_not(eroded_image)
        mask_t=cv2.bitwise_and(dilated_image,dilated_image, mask=mask_inverted)

        # Definir los límites RGB para el color blanco
        lower_white = np.array([0, 0, 0])
        upper_white = np.array([100, 100, 100])
        mask_white  = cv2.inRange(color_image , lower_white, upper_white)
        mask_c=cv2.bitwise_and(mask_t,mask_t, mask=mask_white)
        
        color_image_result_1 = cv2.bitwise_and(color_image ,color_image , mask=mask_c)
        depth_image_result_1 = cv2.bitwise_and(depth_image ,depth_image, mask=mask_c)
        color_image_result_2 = cv2.bitwise_and(color_image ,color_image , mask=eroded_image  )
        depth_image_result_2 = cv2.bitwise_and(depth_image ,depth_image, mask=eroded_image  )

        color_image_result = cv2.add(color_image_result_1,color_image_result_2)
        depth_image_result = cv2.add(depth_image_result_1,depth_image_result_2)


    intrinsic = getIntrisic(sensor+'/intrinsic.json')
    depth = o3d.geometry.Image(depth_image_result)
    color = o3d.geometry.Image(color_image_result)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale=10000, convert_rgb_to_intensity=False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

    
    return pcd


frame_ini=0
frame_fin=1798
step=1
k_list=[1,2,3,4,5,6]

filtro=3
    #1 filtro estandar filtra blancos(nube completa)
    #2 filtro de profundidad (por nube de camara)
    #3 filtro de bordes blancos y profundidad (por nube de camara)
bla= 100 #Controla el filtro de blancos

carpeta = "Final"

T1 = np.load("./Final_Ma/pcd1.npy")
T2 = np.load("./Final_Ma/pcd2.npy")
T3 = np.load("./Final_Ma/pcd3.npy")
T4 = np.load("./Final_Ma/pcd4.npy")
T5 = np.load("./Final_Ma/pcd5.npy")
T6 = np.load("./Final_Ma/pcd6.npy")
#T7 = np.load("./Final_Ma/pcd7.npy")

    # Verificar si la carpeta ya existe
if not os.path.exists(carpeta):
        # Si no existe, crear la carpeta
        os.makedirs(carpeta)

for j in range(frame_ini,frame_fin+step,step):
    
      
    pcdto=getPCD(j,k_list,filtro,T1,T2,T3,T4,T5,T6)
            
     # Definir la caja de límites para el "crop"
    bbox = o3d.geometry.AxisAlignedBoundingBox(
    min_bound=(-0.24307954,-0.21514544,0.08869201),
    max_bound=(0.10651445, 0.21558711,0.43706584))

    # Realizar el "crop" en la nube de puntos
    cropped_pcd = pcdto.crop(bbox)
    #cropped_pcd = pcdto

    # Obtener la matriz RGB de la nube de puntos

    if filtro==1:

        colors = np.asarray(cropped_pcd.colors)
        rgb = (colors[:, 0:3] * 255).astype(np.uint8)

        # Definir los límites RGB para el color blanco
        lower_white = np.array([0, 0, 0])
        upper_white = np.array([bla, bla, bla])

        # Crear una imagen a partir de la matriz RGB
        image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # Aplicar la segmentación de color para detectar los puntos blancos
        mask = cv2.inRange(image, lower_white, upper_white)
            
        # Filtrar los puntos blancos en la nube de puntos
        pcd_white = cropped_pcd.select_by_index(np.where(mask)[0])
        cropped_pcd =pcd_white


    # Crear objeto filtro SOR
    sor = o3d.geometry.PointCloud()
    sor, _ = cropped_pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=2)
    sor = sor.voxel_down_sample(voxel_size=0.001)
    #voxel =0.0015 300 kbytes

      
    o3d.io.write_point_cloud("Final/Frame_Final_"+str(j)+".ply",sor)
    print("Frame: "+str(j)+" of "+str(frame_fin))
