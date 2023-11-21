from djitellopy import Tello
import pandas as pd
import math
import time
import numpy as np
import cv2
import os
import subprocess
import logging

tello = Tello()
# print(tello.get_battery)
tello.connect()
tello.takeoff()
# tello.rotate_counter_clockwise(90)

z = tello.get_height()
approx_window_location = [0, 0, 50]

#go to the Gap
tello.go_xyz_speed(int(approx_window_location[1]), int(-approx_window_location[0]) , int(approx_window_location[2]), 20)        

tello.streamon()
time.sleep(2)
frame_read = tello.get_frame_read()
time.sleep(2)
        
#Set the image path 
imgpath1 = "/home/pear/Desktop/AerialRobotics/apairaikar_p4/pytorch-spynet-master/images/new_images/inputs/one.png"
cv2.imwrite(imgpath1, frame_read.frame)
print("1")
tello.go_xyz_speed(0, -25 , 0, 20)   
# time.sleep(1)
tello.go_xyz_speed(0, 20 , 0, 20)   
imgpath2 = "/home/pear/Desktop/AerialRobotics/apairaikar_p4/pytorch-spynet-master/images/new_images/inputs/two.png"
print("2")
cv2.imwrite(imgpath2, frame_read.frame)

command = f"python3 /home/pear/Desktop/AerialRobotics/apairaikar_p4/pytorch-spynet-master/run.py"
subprocess.run(command, shell=True)
print("RUN")
command = f"python3 /home/pear/Desktop/AerialRobotics/apairaikar_p4/pytorch-spynet-master/visualize.py" #complete_contours.py "
subprocess.run(command, shell=True)
print("VIZ")
#flags for the drone to move
movexy = False
movex = False
movey = False
# Load the image
# image_path = '/home/anuj/Desktop/spynet-pytorch-main/pytorch-spynet-master/1_contours.png''
image_path = '/home/pear/Desktop/AerialRobotics/apairaikar_p4/pytorch-spynet-master/images/new_images/outputs/flow.png'
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

move = [cX - center[0] , cY - center[1]]
if move[0]> 20 and move[1] > 20:
    movexy = True
    print("Move in X-Y positive direction")
    # tello.go_xyz_speed(move[1], move[0] 20 , 0, 20)   
elif move[0] > 20:
    movex == True
    print("Move in positive y")
    # tello.go_xyz_speed(0, move[0] , 0, 20)     
elif move[1] > 20:
    movey == True
    print("move in positive z")
    # tello.go_xyz_speed(move[1], 0 , 0, 20)   
elif move[0]< -20 and move[1] < -20:
    movexy = True
    print("Move in y-z negative direction")
    # tello.go_xyz_speed(move[1], 0, move[0], 20)   
elif move[0] < -20:
    movex == True
    print("Move in negative y")
    # tello.go_xyz_speed(move[0], 0 , 0, 20)   
elif move[1] < -20:
    movey == True
    print("move in negative z")
    # tello.go_xyz_speed(0, move[1] , 0, 20)   
else:
    Donot_move = True
    print("Do not move")
# Save or display the image with the largest contour filled and centroid marked
filled_contours_path = image_path.replace('.png', '_filled_largest_contour_centroid.png')
cv2.imwrite(filled_contours_path, image)

# Optionally, display the image
cv2.imshow('Filled Largest Contour with Centroid', image)
cv2.waitKey(100000)
cv2.destroyAllWindows()
        

tello.land()

