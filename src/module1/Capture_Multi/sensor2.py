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

import subprocess


sys.path.insert(1, '../')

class sensor():

    def __init__(self, device_name, device_index):

        self.device_name = device_name
        self.device_index = device_index

        # Ruta de la carpeta que desea crear
        self.path_folder_device = './'+ self.device_name
        self.path_folder_depth_img = './'+self.device_name+'/'+'depth'
        self.path_folder_color_img = './'+self.device_name+'/'+'color'

        self.create_folders()


    def create_folders(self):

        # Verificar si la carpeta ya existe
        if not os.path.exists(self.path_folder_device):
            # Si no existe, crear la carpeta
            os.makedirs(self.path_folder_device)

        # Verificar si la carpeta ya existe
        if not os.path.exists(self.path_folder_depth_img):
            # Si no existe, crear la carpeta
            os.makedirs(self.path_folder_depth_img)

        # Verificar si la carpeta ya existe
        if not os.path.exists(self.path_folder_color_img):
            # Si no existe, crear la carpeta
            os.makedirs(self.path_folder_color_img)


    def getIntrinsic(self, intrinsicpath,carpeta):
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


    def captureFromSensor(total_frames):

        #Inicialice device
        import pykinect_azure as pykinect

        # Initialize the library, if the library is not found, add the library path as argument
        pykinect.initialize_libraries()
        # Modify camera configuration
        device_config = pykinect.default_configuration
        device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
        device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED
        device = pykinect.start_device(device_index=self.device_index, config=device_config)
        

        cameraname = serialCalibrationFile[str(device.get_serialnum())]


        calibration = device.get_calibration()
        print(calibration)

        #cv2.namedWindow(cameraname+'_Color',cv2.WINDOW_NORMAL)
        #cv2.namedWindow(cameraname+'_Depth',cv2.WINDOW_NORMAL)
        #cv2.namedWindow(cameraname+'_Fusion',cv2.WINDOW_NORMAL)
        Intrinsic_m = self.getIntrinsic(cameraname+'_intrinsic.json',carpeta)

        for frame in range(total_frames):

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

'''
device=inicializeSensor(device_index,carpeta)
time.sleep(2)

for j in range(0,n_frame):
    
    result=captureFromSensor(device,carpeta_p,carpeta_c,camera_indexes_i.get(device_index),j)
#print(carpeta_p,device_index)
'''

def get_devices_connected():

    try:
        print("Detecting devices")
        cmd = r'C:\Program Files\Azure Kinect SDK v1.4.1\tools\k4arecorder.exe --list'
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output = result.stdout.decode('utf-8')
        
        devices = {}

        for line in output.split('\n'):
            if line.strip() == '':
                continue
            fields = line.split()
            index = int(fields[0].split(':')[1])
            serial = fields[1].split(':')[1]
            devices[index] = {'serial': serial}
        #print(devices)

    except:
        print("An exception occurred") 

    return devices


if __name__=="__main__":

    with open('cameras.json') as cameras_file:
        cameras_list = cameras_file.read()

    cameras_list_dict = json.loads(cameras_list)
    devices = get_devices_connected()

    print(cameras_list_dict, devices)

    sensores=[]

    for idx in devices.keys():
        serial = devices[idx]["serial"]
        name_camera=cameras_list_dict.get(serial)
        sensores.append(sensor(name_camera, idx))
        print("Camera {} ready detected".format(sensores[idx].device_name))


    


