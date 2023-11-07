from djitellopy import Tello
import pandas as pd
import math
import time
import numpy as np
import cv2
from RRTstar import RRTStar
from helpers import quat2euler 
import os
import subprocess
from three_dim_pose_estimation import get_pose

def follow_path(pathf, R, current_loc):
    df = pd.read_csv(pathf)
    init_x = 0
    init_y  = 0
    init_z = 0
    
    r_d = np.array([[0,-1,0],
                    [1,0,0],
                    [0,0,1]])
    
    for column_index in range(len(df.columns)):
        column_data = df.iloc[:, column_index] 
        x_loc = column_data.iloc[0] -init_x
        y_loc = column_data.iloc[1] -init_y
        z_loc = column_data.iloc[2] -init_z

        inc_coords = (np.array([[x_loc,y_loc,z_loc]])).T
        inc_coords_wf = (R.T)*inc_coords
        
        #Tello cannot move unless the distance specified is greater than 20, in such cases the tello takes the next point which is atleast 20 away from the current x, y and z
        if abs(x_loc) < .20 or abs(y_loc) < .20 or abs(z_loc) < .20 :
            print("Difference too small")
            continue

        else:
            #print(x_loc,y_loc,z_loc)
            go_to_i  = (np.array([x_loc,y_loc,z_loc])).T
            go_to = r_d*go_to_i 
            tello.go_xyz_speed(-int(go_to[0]*100),-int(go_to[1]*100),int(go_to[2]*100), 25)
            init_x = column_data.iloc[0]
            init_y = column_data.iloc[1]
            init_z = column_data.iloc[2]
            current_loc[0] += inc_coords_wf[0]*100
            current_loc[1] += inc_coords_wf[1]*100
            current_loc[2] += inc_coords_wf[2]*100

        window_rotation = False

    return curr_loc

#read the map and store the approx. window coordinates
map_file = '/home/pear/AerialRobotics/apairaikar_p3a/maps'
window_locs = []

with open(map_file, 'r') as file:
    lines = file.readlines()

    for line in lines:
        line_parts = line.strip().split()

        if len(line_parts) > 0:
            if line_parts[0] == "boundary" and len(line_parts) == 7:
                    bxmin, bymin, bzmin, bxmax, bymax, bzmax = map(float, line_parts[1:])
            elif line_parts[0] == "window":
                 x, y, z, xdelta, ydelta, zdelta, qw, qx, qy, qz, xangdelta, yangdelta, zangdelta = map(float, line_parts[1:])
                 window_locs.append([x,y,z])
                 
tello = Tello()

tello.connect()
tello.takeoff()

z = tello.get_height()

init_x = 0
init_y  = 0
init_z = 0

real_x = 0
real_y = 0 
real_z = 0
curr_loc = [0,0,0]
window_rotation =  False
go_through = False
for i in range(len(window_locs)):

        
        approx_window_location = [window_locs[i][0] - 25, window_locs[i][1] - 25, window_locs[i][2]]
        path = RRTStar(curr_loc, approx_window_location,bxmin, bymin, bzmin, bxmax, bymax, bzmax)
        
        if window_rotation == False:
            R = np.eye(3)

        curr_loc = follow_path(path,R,curr_loc)
        
        #change the orientation of the drone to that of the window
        window_orient_approx = quat2euler([qw,qx,qy,qz])
        theta = window_orient_approx[2]
        
        #if the window has certain rotation, find the Rotation matrix of the drone coorinate frame
        if theta != 0 :
            window_rotation = True
            R = np.array([[math.cos(theta),   math.sin(theta),0],
                          [-1*math.sin(theta),math.cos(theta),0],
                          [0                 ,0,              1]])    
            
        if theta > 0:
            tello.rotate_counter_clockwise(theta)
            go_through = True
        
        elif theta < 0:
            tello.rotate_clockwise(theta)
            go_through = True
        
        tello.streamon()
        time.sleep(2)
        frame_read = tello.get_frame_read()
        time.sleep(2)
        
        #Set the image path 
        imgpath = f"./feed/input/in.png"
        infpath = f"./feed/output/out.png"
        cv2.imwrite(imgpath, frame_read.frame)
        ###############################################################################################
        # Run UNet on the image
        # Run traditional CV to get the 3 D coords of the window corners
        # store the 3D coordinates
        ###############################################################################################
        command = f"python predict.py --model ./checkpoints/checkpoint_epoch180.pth -i {imgpath} --viz --output {infpath}"
        subprocess.run(command, shell=True)


        if go_through == True:
            cam_goal_coords = get_pose(infpath)
            cam_goal_coords[1] = cam_goal_coords[1] + 10 
            goal_coords = [cam_goal_coords[0],cam_goal_coords[2],-cam_goal_coords[1]]

        path = RRTStar([0,0,0],[goal_coords],bymin, bzmin, bxmax, bymax, bzmax)
        curr_loc = follow_path(path, R, goal_coords)


tello.land()
