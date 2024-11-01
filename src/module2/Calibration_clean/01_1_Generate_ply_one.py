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
    
    
    if  filtro==0:

        color_image_result = color_image
        depth_image_result = depth_image
    
    elif filtro==1: 

        mask_1=depth_gray
        dilated_image = cv2.dilate(mask_1, kernel, iterations=2)
        eroded_image  = cv2.erode(dilated_image, kernel, iterations=10)
        color_image_result = cv2.bitwise_and(color_image ,color_image , mask=eroded_image)
        depth_image_result = cv2.bitwise_and(depth_image ,depth_image, mask=eroded_image)
    
    elif filtro==2: 
        mask_1= cv2.inRange(depth_gray,1,9)
        eroded_image  = cv2.erode(mask_1, kernel, iterations=6)
        dilated_image = cv2.dilate(eroded_image, kernel, iterations=2)
        
        ret, thresh = cv2.threshold(dilated_image, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        # Encontrar los contornos en la imagen
        contours, hierarchy = cv2.findContours(thresh , cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Ordenar los contornos en orden descendente según su área
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        # Crear un nuevo máscara que contenga solo el contorno de mayor área
        mask_2 = np.zeros_like(mask_1)
        cv2.drawContours(mask_2, [contours[0]], 0, 255, -1)

        if sensor=="k07":

            color_image_result = cv2.bitwise_and(color_image ,color_image , mask=dilated_image)
            depth_image_result = cv2.bitwise_and(depth_image ,depth_image, mask=dilated_image)
        else:
            color_image_result = cv2.bitwise_and(color_image ,color_image , mask=mask_2)
            depth_image_result = cv2.bitwise_and(depth_image ,depth_image, mask=mask_2)
    
    elif filtro==3:
        mask_1= cv2.inRange(depth_gray,1,9)
        eroded_image_a  = cv2.erode(mask_1, kernel, iterations=12)
        dilated_image = cv2.dilate(eroded_image_a, kernel, iterations=12)
        eroded_image  = cv2.erode(dilated_image, kernel, iterations=6)

        mask_inverted = cv2.bitwise_not(eroded_image)
        mask_t=cv2.bitwise_and(dilated_image,dilated_image, mask=mask_inverted)

        # Definir los límites RGB para el color blanco
        lower_white = np.array([0, 0, 0])
        upper_white = np.array([90, 90, 90])
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

#Definir los delay entre camaras
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

#Parametros de calibracion

#Definir el fram de arranque y el frame final
frame_ini=370
frame_final=650
step=10 #cada cuanto voy a generar un frame

k_list=[1,2,3,4,5,6,7] #Segun el numero de camaras

filtro=1
    #0  sin filtro
    #1 filtro estandar filtra blancos(nube completa)
    #2 filtro de profundidad (por nube de camara)
    #3 filtro de bordes blancos y profundidad (por nube de camara)

pcd_dict = {}

carpeta1 = './Points_Cloud/'

# Verificar si la carpeta ya existe
if not os.path.exists(carpeta1):
    # Si no existe, crear la carpeta
    os.makedirs(carpeta1)

pcdto=o3d.geometry.PointCloud()

for w in range (frame_ini,frame_final+step,step):
  
  carpeta2 = 'Points_Cloud/Frame_'+str(w)
  print("Frame:"+str(w))

    # Verificar si la carpeta ya existe
  if not os.path.exists(carpeta2):
        # Si no existe, crear la carpeta
        os.makedirs(carpeta2)

  for i in k_list:
    
    sensor='k0'+str(i)
    frame = str(int(w)+dictOffsets[sensor]).zfill(5)
    pcd_name = "pcd{}".format(i)
    pcd_dict[pcd_name]=create3DFromImages_Filter_Border(sensor,frame,filtro)
  
    pcdto=pcd_dict[pcd_name]
            
     # Definir la caja de límites para el "crop"
    bbox = o3d.geometry.AxisAlignedBoundingBox(
    min_bound=(-0.11973586,-0.11016178,0.06848225),    #min corner cloudcompare
    max_bound=(0.05660336, 0.08104238,0.20232895))     #max corner cloudcompare

    # Realizar el "crop" en la nube de puntos
    cropped_pcd = pcdto.crop(bbox)
    #cropped_pcd = pcdto

    if filtro==1:

        colors = np.asarray(cropped_pcd.colors)
        rgb = (colors[:, 0:3] * 255).astype(np.uint8)

        # Definir los límites RGB para el color blanco
        lower_white = np.array([0, 0, 0])
        upper_white = np.array([150, 150, 150]) #Filtro de blancos  depende de los blancos

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
    #sor = sor.voxel_down_sample(voxel_size=0.0001)

    o3d.io.write_point_cloud("Points_Cloud/Frame_"+str(w)+"/k0"+str(i)+".ply",sor)
    print("k0"+str(i)+" of "+str(len(k_list)))
   

