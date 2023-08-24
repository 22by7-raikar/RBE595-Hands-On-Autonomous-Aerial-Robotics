import numpy as np
import cv2
import bpy
import os
import copy
import math
import apriltag

class state_machine:
    def __init__(self):
        # User code executes every dt time in seconds
        self.dt = 0.050

        # location of each tag
        self.waypoints = [
            [-1.7,      0.8,    1.5],
            [-1.2,     -1.6,      1.5],
            [0.9,     0.7,    1.5]
        ]

        # Current waypoint
        self.currentWP = [0,    -0.2,    1.5]

    def step(self, time, currpos):
        """
        Input: time, current position in blender frame (x, y, z)
        Output: desired position
            Si unit unless specified otherwise

            FILL YOUR CODE HERE!
        """

        # Write your logic here. Do not edit anything else apart from paths.
        if time < 1:
            xyz_desired = self.currentWP
            return xyz_desired
        
        num_waypts = len(self.waypoints)
        # atwWP = 0
        landed = False
        i = 0
        curr_waypt = self.waypoints[i]
        cwp_x, cwp_y, cwp_z, = curr_waypt[0], curr_waypt[1], curr_waypt[2]
        cp_x, cp_y, cp_z, = currpos[0], currpos[1], currpos[2]

        euc_dis = math.sqrt(((cp_x - cwp_x)**2) + ((cp_y - cwp_y)**2) + ((cp_z - cwp_z)**2))
        threshold = 1.4
        if euc_dis < threshold:
            if landed:
                landed = False
                return [cwp_x, cwp_y, 1.5]
            
            #Hasn't Landed yet OR risen from landing
            curr_img = self.fetchLatestImage                        #Get image 
            gray_img = cv2.cvtColor(curr_img, cv2.COLOR_BGR2GRAY)   #convert to grayscale
            detector = apriltag.Detector()  
            result = detector.detect(gray_img)
            t_num = result[0].tag_id
            
            if t_num == 4:
                landed = True
                curr_time = time
                xyz_desired = [cwp_x, cwp_y, 0.1]
            
            i = i+1
        
        return xyz_desired

    def fetchLatestImage(self):
        # Fetch image - renders the camera, saves the rendered image to a file and reads from it. 
        path_dir = bpy.data.scenes["Scene"].node_tree.nodes["File Output"].base_path

        # Render Drone Camera
        cam = bpy.data.objects['DownCam']    
        bpy.context.scene.camera = cam
        bpy.context.scene.render.filepath = os.path.join(path_dir, 'DownCam_latest.png')
        bpy.ops.render.render(write_still=True)

        return cv2.imread(bpy.context.scene.render.filepath)
