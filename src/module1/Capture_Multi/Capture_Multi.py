import subprocess
import os
import keyboard
import time
import cv2
import multiprocessing
import time

# Lista de scripts a ejecutar
n_frame=20

arg0 = str(0)
arg1 = str(1)
arg2 = str(2)
arg3 = str(3)
#arg4 = str(4)
#arg5 = str(5)
#arg6 = str(6)


# Lista para almacenar los procesos de cada script
procesos = []   
scripts = [
            ('capture_0.py', arg0,str(n_frame)),
            ('capture_1.py', arg1,str(n_frame)),
            ('capture_2.py', arg2,str(n_frame)),
            ('capture_3.py', arg3,str(n_frame)),
            #('capture_4.py', arg4,str(n_frame)),
            #('capture_5.py', arg5,str(n_frame))

            #('capture_6.py', arg6, argp)
        ]

time.sleep(5)
# Ejecutar cada script en un proceso separado
for script in scripts:
    
    comando = ['python', script[0],script[1],script[2]] 
    proceso = subprocess.Popen(comando, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    procesos.append(proceso)
    
#    print(fin-inicio)

# Esperar a que todos los procesos terminen
for proceso in procesos:
    proceso.wait()

# Obtener la salida de cada proceso
for proceso in procesos:
    salida, errores = proceso.communicate()
    print(salida.decode('utf-8'))
    print(errores.decode('utf-8'))

#fin = time.time()
#print(fin-inicio)

