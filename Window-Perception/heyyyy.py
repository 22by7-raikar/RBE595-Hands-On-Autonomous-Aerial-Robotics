from djitellopy import Tello
import pandas as pd
import math
import time
import RRTstar
import numpy as np
import cv2

def follow_path(path,R, current_loc):
    df = pd.read_csv('/home/pear/Downloads/testtraj.csv')
    init_x = 0
    init_y  = 0
    init_z = 0
    for column_index in range(len(df.columns)):
        column_data = df.iloc[:, column_index]
        #print(column_data.iloc[0],column_data.iloc[1],column_data.iloc[2])
        
        #Sending incremental co-odinates to the drone    
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
            tello.go_xyz_speed(-int(x_loc*100),-int(y_loc*100),int(z_loc*100), 25)
            init_x = column_data.iloc[0]
            init_y = column_data.iloc[1]
            init_z = column_data.iloc[2]
            current_loc[0] += inc_coords_wf[0]
            current_loc[1] += inc_coords_wf[1]
            current_loc[2] += inc_coords_wf[2]

#read the map and store the approx. window coordinates
map_file = cfg['env']['map_file']
env_file = os.path.expanduser(os.path.join(env_path, map_file))
window_locs = []
with open(env_file, 'r') as file:
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
window_rotation =  False
for i in range(len(window_locs)):

        curr_loc = [real_x,real_y,real_z]
        approx_window_location = [window_locs[i][0] - 25, window_locs[i][1] - 25, window_locs[i][2]]
        path = RRTstar(curr_loc, approx_window_location,bxmin, bymin, bzmin, bxmax, bymax, bzmax)
        if window_rotation == False:
         R = np.eye(3)
        follow_path(path,R,curr_loc)
        #change the orientation of the drone to that of the window
        window_orient_approx = quat2euler([qw,qx,qy,qz])
        theta = window_orient_approx[2]
        #if the window has certain rotation, find the Rotation matrix of the drone coorinate frame
        if theta != 0 :
            window_rotation == True
            R = np.array([[cos(theta), sin(theta),0],
                          [-1*sin(theta),cos(theta),0],
                          [0,0,1]])    
        if theta > 0:
            tello.rotate_counter_clockwise(theta)
        elif theta < 0:
            window_rotation == True
            tello.rotate_clockwise(theta)
        
        tello.streamon()
        time.sleep(2)
        frame_read = tello.get_frame_read()
        time.sleep(2)
        #Set the image path 
        imgpath = f"outputs/img{count}.png"
        cv2.imwrite("picture.png", frame_read.frame)
        ###############################################################################################
        # Run UNet on the image
        # Run traditional CV to get the 3 D coords of the window corners
        # store the 3D coordinates
        ###############################################################################################

        path = RRTstar([0,0,0],[goal_coords])





tello.land()
