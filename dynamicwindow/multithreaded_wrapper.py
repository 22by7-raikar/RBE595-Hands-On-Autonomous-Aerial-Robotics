from djitellopy import Tello
import pandas as pd
import math
import time
import numpy as np
import cv2
import os
import subprocess
import logging
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import struct
from three_dim_pose_estimation import get_pose
import threading
import queue

def unet_prediction_and_pose_estimation(tello, imgpath, infpath, command):
    tello.streamon()
    frame_read = tello.get_frame_read()
    cv2.imwrite(imgpath, frame_read.frame)
    tello.streamoff()
    subprocess.run(command, shell=True)
    cam_goal_coords = get_pose(infpath)

    return cam_goal_coords


def get_optflow_pics(tello, imgpath1, imgpath2, infpath, command):
    tello.streamon()
    cv2.imwrite(imgpath1, frame_read.frame)
    frame_read = tello.get_frame_read()
    tello.go_xyz_speed(0, -25 , 0, 50)  
    tello.go_xyz_speed(0, 25 , 0, 50)  
    cv2.imwrite(imgpath2, frame_read.frame)
    tello.streamoff()

def main():

    window_locs = []
    window_locs.append([245,90,140]) #[-0.5, 1.05, 1.30]
    window_locs.append([425,-125,140]) #[0.55, 3, 1.30]

    opticalflow_window_loc = [0, 5.25, 0.40]
    curr_loc = [0, 0, tello.get_height()]

    tello = Tello()
    tello.connect()
    tello.takeoff()

    for i in enumerate(window_locs):

        print("z",window_locs[i][2])
        if i == 0:
            tello.go_xyz_speed(int(window_locs[i][0]-curr_loc[0]) - 200, int(window_locs[i][1]-curr_loc[1]) -20  , int(window_locs[i][2]-curr_loc[2]) , 75)        
            curr_loc[0] += window_locs[i][0] - 200
            curr_loc[1] += window_locs[i][1] 
            curr_loc[2] += window_locs[i][2] #- curr_loc[2]

            # tello.go_xyz_speed(205, -20, -25, 75)

        else:
            if (window_locs[i][1]-curr_loc[1]) > 500: 
                y = 500
            else:
                y = window_locs[i][1]-curr_loc[1]

            tello.go_xyz_speed(int(window_locs[i][0]-curr_loc[0]) - 180, int(y)  , int(window_locs[i][2]-curr_loc[2])  , 75)
            curr_loc[0] += window_locs[i][0]-curr_loc[0] - 180
            curr_loc[1] += y
            curr_loc[2] += window_locs[i][2]-curr_loc[2] 

        print("near window:", curr_loc)

        # Set image paths for UNet and pose estimation
        imgpath = f"/path/to/save/window_{i}_image.jpg"
        infpath = f"/path/to/save/window_{i}_inference_output.png"
        command = f"python /home/pear/Desktop/WindowDetection-main/UNet_model/predict.py --model /home/pear/Desktop/WindowDetection-main/UNet_model/checkpoints/checkpoint_epoch25.pth -i {imgpath} --viz --output {infpath}"

        # Run UNet prediction and pose estimation
        cam_goal_coords = unet_prediction_and_pose_estimation(tello, imgpath, infpath, command)
        print("Camera goal coordinates for window", i, ":", cam_goal_coords)

    #Set the image path 
    
    imgpath1 = "/home/pear/Desktop/WindowDetection-main/SPyNet_model/images/new_images/inputs/one.png"
    print("1")

    print("2")
    movement_x, movement_y = get_optflow_pics(tello, imgpath1, imgpath2)

    # time.sleep(1)
    tello.go_xyz_speed(0, 25 , 0, 50)   
    imgpath2 = "/home/pear/Desktop/WindowDetection-main/SPyNet_model/images/new_images/inputs/two.png"


    tello.land()

if __name__ == "__main__":
    main()
