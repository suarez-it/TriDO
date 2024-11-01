import open3d as o3d
#import time 
from time import sleep
import numpy as np
from scipy.spatial.transform import Rotation

frames =1800
frame_ini=0
frame_m=550

vis = o3d.visualization.Visualizer()
vis.create_window()

#render_option = o3d.visualization.RenderOption(point_size=1.0)
render_option = vis.get_render_option()
render_option.point_size = 1.5


pcd = o3d.io.read_point_cloud(f'Final\Frame_Final_'+str(frame_m)+'.ply')
vis.add_geometry(pcd)
vis.poll_events()

view_control = vis.get_view_control()

#vis.get_render_option().load_from_json("./render_options.json")'
vis.update_renderer()

link = ".\Final\Frame_Final_"
#time.sleep(2)
#sleep(2)

###----------------------------------------------------------------
for i in range(frame_ini, frames+1):

    link2 = link + str(i)+ ".ply"

    print(link2)
     
    pcd.points = o3d.io.read_point_cloud(link2).points
    pcd.colors = o3d.io.read_point_cloud(link2).colors
    
    # Rotar la nube de puntos en 90 grados en torno a los ejes X, Y y Z
    rot_x = Rotation.from_euler('x', 0, degrees=True).as_matrix()
    rot_y = Rotation.from_euler('y', 0, degrees=True).as_matrix()
    rot_z = Rotation.from_euler('z', -90, degrees=True).as_matrix()

    pcd.rotate(rot_x, center=pcd.get_center())
    pcd.rotate(rot_y, center=pcd.get_center())
    pcd.rotate(rot_z, center=pcd.get_center())
    
    #vis.draw_geometries([pcd])
    vis.update_geometry(pcd)

    #vis.get_render_option().load_from_json("./render_options.json")

    vis.poll_events()
    vis.update_renderer()