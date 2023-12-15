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

window_locs = []
window_locs.append([-0.9, 2.45, 1.40]) #[-0.5, 1.05, 1.30]
window_locs.append([1.25, 4.25, 1.40]) #[0.55, 3, 1.30]
tello = Tello()

tello.connect()
tello.takeoff()
time.sleep(2)
z = tello.get_height()
print("this is height :z",z)
init_x = 0
init_y  = 0

real_x = 0
real_y = 0 
real_z = 0
curr_loc = [0,0,0]
window_rotation =  False
go_through = False

# tello.move_forward(300)
for i in range(len(window_locs)):
        #approx_window_location = [window_locs[i][0] - 25, window_locs[i][1] - 25, window_locs[i][2]]
        #go to the window
        window_locs[i][0] = window_locs[i][0] - curr_loc[0] 
        window_locs[i][1] = window_locs[i][1] - curr_loc[1] 
        window_locs[i][2] = window_locs[i][2] - curr_loc[2]

        print("z",window_locs[i][2])
        if i == 0:
            tello.go_xyz_speed(int(window_locs[i][1]*100) - 200, int(-window_locs[i][0]*100) - 10   , int(window_locs[i][2])*100 - 20, 75)        
            curr_loc[0] += window_locs[i][0] + 0.20
            curr_loc[1] += window_locs[i][1] - 2
            curr_loc[2] += window_locs[i][2] 

            # tello.go_xyz_speed(205, -20, -25, 75)

        else:
            tello.go_xyz_speed(int(window_locs[i][1]*100) - 180, int(-window_locs[i][0]*100) + 35 , int(window_locs[i][2])*100 + 20 , 75)
            curr_loc[0] += window_locs[i][0] - 0.35
            curr_loc[1] += window_locs[i][1] - 1.8
            curr_loc[2] += window_locs[i][2] 
        print("near window:", curr_loc)

        tello.streamon()
        time.sleep(2)
        frame_read = tello.get_frame_read()
        time.sleep(2)
        
        #Set the image path 
        imgpath = "/home/pear/Desktop/WindowDetection-main/inputs/image.jpg"
        infpath = "/home/pear/Desktop/WindowDetection-main/outputs/out.png"
        cv2.imwrite(imgpath, frame_read.frame)
        tello.streamoff()
        command = f"python /home/pear/Desktop/WindowDetection-main/UNet_model/predict.py --model /home/pear/Desktop/WindowDetection-main/UNet_model/checkpoints/checkpoint_epoch25.pth -i {imgpath} --viz --output {infpath}"
        subprocess.run(command, shell=True)


        cam_goal_coords = get_pose(infpath)
        cam_goal_coords[1] = cam_goal_coords[1] + 10 
        goal_coords = [cam_goal_coords[2],-cam_goal_coords[0],-cam_goal_coords[1]]
        print(goal_coords)

        if goal_coords[0] < 20:
             goal_coords[0] = 0
        if goal_coords[1] < 20:
            goal_coords[1] = 0
        if goal_coords[2] < 20:
            goal_coords[2] = 0   

        if goal_coords[0]+135 > 500:
            goal_coords[0] = goal_coords[0] -135

        tello.go_xyz_speed(((int(goal_coords[0])+135)) , int(goal_coords[1])- 20 , int(goal_coords[2])-25, 50)
  

      
        curr_loc[1] += goal_coords[0]/100+1.35
        curr_loc[0] += -goal_coords[1]/100-0.20
        curr_loc[2] += goal_coords[2]/100-0.25

        print("after unet",curr_loc)


#OPTICAL FLOW

def read_flo_file(file_path):
    with open(file_path, 'rb') as file:
        # Read the header
        header = file.read(4)
        if header != b'PIEH':
            raise ValueError("Invalid .flo file")

        width = struct.unpack('i', file.read(4))[0]
        height = struct.unpack('i', file.read(4))[0]

        # Read the flow data
        data = np.zeros((height, width, 2), dtype=np.float32)
        for y in range(height):
            for x in range(width):
                data[y, x, 0] = struct.unpack('f', file.read(4))[0]
                data[y, x, 1] = struct.unpack('f', file.read(4))[0]

    return data

def visualize_flow(flow_data,):
    # Ensure the flow_data has the correct shape (height, width, 2)
    if flow_data.ndim != 3 or flow_data.shape[2] != 2:
        raise ValueError("Invalid flow_data shape. It should have shape (height, width, 2).")

    height, width, _ = flow_data.shape

    # Calculate the magnitude of flow vectors
    magnitude = np.sqrt(flow_data[:, :, 0] ** 2 + flow_data[:, :, 1] ** 2)

    # Create a colormap for flow magnitude
    cmap = plt.get_cmap('gray')
    norm = plt.Normalize(vmin=magnitude.min(), vmax=magnitude.max())
    
    # Apply the colormap to the magnitudes
    flow_color = cmap(norm(magnitude))
    
    save_path = '/home/pear/Desktop/WindowDetection-main/SPyNet_model/images/new_images/outputs/flow.png'  # Prev 1.png Update this path as needed 
    print("image saved")
    plt.savefig(save_path, bbox_inches='tight', pad_inches=0)
    plt.close()

    # Optionally, display the image
    saved_image = plt.imread(save_path)

    # Load the saved image
    saved_image = cv2.imread(save_path)

    # Convert to grayscale for contour detection
    gray_image = cv2.cvtColor(saved_image, cv2.COLOR_BGR2GRAY)

    # Use Canny edge detection
    edges = cv2.Canny(gray_image, threshold1=45, threshold2=150)

    # Find contours from Canny edges
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours
    contour_image = cv2.drawContours(saved_image.copy(), contours, -1, (0, 0, 0), 4)  # Green contours
    
    # Save or display the image with contours
    contour_path = save_path.replace('.png', '_contours.png')
    cv2.imwrite(contour_path, contour_image)


z = tello.get_height()
approx_window_location = [-0.30, 5.25, 0.40]
print("this is shit:",approx_window_location[1]-curr_loc[1], -approx_window_location[0]+curr_loc[0]*100 , approx_window_location[2]-curr_loc[2])
#go to the Gap
tello.go_xyz_speed(int((approx_window_location[1])-curr_loc[1])*100, int((-approx_window_location[0])+curr_loc[0])*100 , int((approx_window_location[2])-curr_loc[2])*100, 50)        

tello.streamon()
time.sleep(2)
frame_read = tello.get_frame_read()
time.sleep(2)
        
#Set the image path 
imgpath1 = "/home/pear/Desktop/WindowDetection-main/SPyNet_model/images/new_images/inputs/one.png"
cv2.imwrite(imgpath1, frame_read.frame)
print("1")
tello.go_xyz_speed(0, -25 , 0, 50)   
# time.sleep(1)
tello.go_xyz_speed(0, 25 , 0, 50)   
imgpath2 = "/home/pear/Desktop/WindowDetection-main/SPyNet_model/images/new_images/inputs/two.png"
print("2")
cv2.imwrite(imgpath2, frame_read.frame)

#time.sleep(200)
tello.go_xyz_speed(-125, 0 , 0, 100)
command = f"python3 /home/pear/Desktop/WindowDetection-main/SPyNet_model/run.py"
subprocess.run(command, shell=True)
tello.go_xyz_speed(125, 0 , 0, 100)
command = f"python3 /home/pear/Desktop/WindowDetection-main/SPyNet_model/visualize.py" #complete_contours.py "
subprocess.run(command, shell=True)
#flags for the drone to move
movexy = False
movex = False
movey = False
# Load the image
# image_path = '/home/anuj/Desktop/spynet-pytorch-main/pytorch-spynet-master/1_contours.png''
image_path = '/home/pear/Desktop/WindowDetection-main/SPyNet_model/images/new_images/outputs/flow_contours.png'
image = cv2.imread(image_path)
print("FLOW")
center = image.shape[1]/2,image.shape[0]/2
# Check if the image has been loaded correctly
if image is None:
    raise ValueError("Could not read the image.")

# Convert to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Threshold the image to get a binary mask
# Assuming the contours are in black
_, binary_mask = cv2.threshold(gray, 15, 255, cv2.THRESH_BINARY_INV)

# Find contours
contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Find the largest contour by area
largest_contour = max(contours, key=cv2.contourArea)

# Create a new black image of the same size
filled_largest_contour = np.zeros_like(gray)

# Fill the largest contour with white
cv2.drawContours(filled_largest_contour, [largest_contour], -1, 255, thickness=cv2.FILLED)

# Calculate the moments of the largest contour
M = cv2.moments(largest_contour)
if M["m00"] != 0:
    # Calculate centroid
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
else:
    # Set centroid to the center of the image
    cX, cY = center
 
# Mark the centroid on the image
cv2.circle(image, (cX, cY), 15, (0, 0, 255), -1)  # Red dot
print(cX,cY)
print(center)
print("CENTROID")

# Save or display the image with the largest contour filled and centroid marked
filled_contours_path = image_path.replace('.png', '_filled_largest_contour_centroid.png')
cv2.imwrite(filled_contours_path, image)

move = [cX - center[0] , cY - center[1]]
if move[0]> 20 and move[1] > 20:
    movexy = True
    print("Move in X-Y positive direction")
    tello.go_xyz_speed(0, 20, 20, 45)   
    # tello.go_xyz_speed(move[1], move[0] 20 , 0, 20)   
elif move[0] > 20:
    movex == True
    print("Move in positive z")
    tello.go_xyz_speed(0, -25 , -20, 45)     
elif move[1] > 20:
    movey == True
    print("move in positive z")
    # tello.go_xyz_speed(move[1], 0 , 0, 20)   
elif move[0]< -20 and move[1] < -20:
    movexy = True
    print("Move in y-z negative direction")
    tello.go_xyz_speed(0, -25, -20, 45)   
elif move[0] < -20:
    movex == True
    print("Move in negative y")
    # tello.go_xyz_speed(move[0], 0 , 0, 20)   
elif move[1] < -20:
    movey == True
    print("move in negative z")
    # tello.go_xyz_speed(0, 0, move[1], 20)   
else:
    Donot_move = True
    print("Do not move")

tello.go_xyz_speed(350, 0, 0, 100)  

tello.land()
