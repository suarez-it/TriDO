import sys
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
import time
import os
import keyboard
import gc

import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_face_mesh = mp.solutions.face_mesh


sys.path.insert(1, '../')
import pykinect_azure as pykinect


def getIntrinsic(intrinsicpath,carpeta):
    intrinsic = None
    with open(intrinsicpath) as f:
        data = json.load(f)
        width = data['width']
        height = data['height']
        intrinsic_matrix = data['intrinsic_matrix']
        fx, fy, cx, cy = intrinsic_matrix[0], intrinsic_matrix[4], intrinsic_matrix[6], intrinsic_matrix[7]
        intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    
    with open(carpeta+'/'+'intrinsic.json', 'w') as fp:
       json.dump(data, fp)
       
    return intrinsic


def inicializeSensor(device_index,carpeta):
    # Initialize the library, if the library is not found, add the library path as argument
    pykinect.initialize_libraries()
    # Modify camera configuration
    device_config = pykinect.default_configuration
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
    device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED
    device = pykinect.start_device(device_index=device_index, config=device_config)
    #bodyTracker = pykinect.start_body_tracker()
    cameraname = serialCalibrationFile[str(device.get_serialnum())]

    #cv2.namedWindow(cameraname+'_Body',cv2.WINDOW_NORMAL)
    #cv2.namedWindow(cameraname+'_Color',cv2.WINDOW_NORMAL)
    #cv2.namedWindow(cameraname+'_Depth',cv2.WINDOW_NORMAL)
    #cv2.namedWindow(cameraname+'_Fusion',cv2.WINDOW_NORMAL)
    Intrinsic_m = getIntrinsic(cameraname+'_intrinsic.json',carpeta)


    
    ####################################################################
    # Get capture
    #with mp_face_mesh.FaceMesh(
        #max_num_faces=1,
        #refine_landmarks=True,
        #min_detection_confidence=0.5,
        #min_tracking_confidence=0.5) as face_mesh:
    
    return device




def captureFromSensor(device,carpeta_p,carpeta_c,index,frame):
    
    while True:
        capture = device.update()
        #body_frame = bodyTracker.update()

        # Get the color image from the capture 1
        ret_color, color_image_temp = capture.get_color_image()
        ret_depth, depth_image = capture.get_transformed_depth_image()
        #ret_depth_transformed, transformed_colored_depth_image = capture.get_transformed_colored_depth_image()

        #ret, body_image_color = body_frame.get_transformed_segmentation_image()

        if not ret_color or not ret_depth:
            continue
        else:
            cv2.imwrite(carpeta_p+'/'+str(frame).zfill(5)+'.png',depth_image)
            color_image = color_image_temp.copy()
            color_image[:,:,0], color_image[:,:,2] = color_image_temp[:,:,2], color_image_temp[:,:,0]
            color_image  = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            cv2.imwrite(carpeta_c+'/'+str(frame).zfill(5)+'.jpg',color_image)
            break
            


    #device.close()
    # Press q key to continue
    #del device
    #########################################################3
    return True


##########################################
### IMPORTANT
# Deduce index and serial from sensors
serialCalibrationFile = {
    '000472115012': 'k01', #3
    '000128315012': 'k02', #0
    '000423514612': 'k03', #2
    '000343414612': 'k04', #5
    '000107615012': 'k05', #1
    '000451415012': 'k06', #4
    #'000175114612': 'k07' #3
}
camera_indexes = {
    'k01':3,
    'k02':0,
    'k03':2,
    'k04':5,
    'k05':1,
    'k06':4
    #'k07':3
}

camera_indexes_i = {
    3:'k01',
    0:'k02',
    2:'k03',
    5:'k04',
    1:'k05',
    4:'k06'
    #3:'k07'
}

device_index = int(sys.argv[1])
n_frame = int(sys.argv[2])

# Ruta de la carpeta que desea crear
carpeta = './'+camera_indexes_i.get(device_index)
carpeta_p = './'+camera_indexes_i.get(device_index)+'/'+'depth'
carpeta_c = './'+camera_indexes_i.get(device_index)+'/'+'color'

# Verificar si la carpeta ya existe
if not os.path.exists(carpeta):
    # Si no existe, crear la carpeta
    os.makedirs(carpeta)

# Verificar si la carpeta ya existe
if not os.path.exists(carpeta_p):
    # Si no existe, crear la carpeta
    os.makedirs(carpeta_p)

# Verificar si la carpeta ya existe
if not os.path.exists(carpeta_c):
    # Si no existe, crear la carpeta
    os.makedirs(carpeta_c)


device=inicializeSensor(device_index,carpeta)
time.sleep(2)


for j in range(0,n_frame):
    
    result=captureFromSensor(device,carpeta_p,carpeta_c,camera_indexes_i.get(device_index),j)
#print(carpeta_p,device_index)