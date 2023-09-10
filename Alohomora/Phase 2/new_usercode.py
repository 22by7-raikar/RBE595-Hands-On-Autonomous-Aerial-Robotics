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
        self.i = 0
        self.landed = False
        self.t = 0
        
        # location of each tag
        self.waypoints = [
            [-1.7, 0.8, 1.5],
            [-1.2, -1.6, 1.5],
            [0.9, 0.7, 1.5]
        ]

        # Current waypoint
        self.currentWP = [0, -0.2, 1.5]

    def step(self, time, currpos):
        """
        Input: time, current position in blender frame (x, y, z)
        Output: desired position
        Si unit unless specified otherwise
        """

        xyz_desired = self.currentWP
        curr_waypt = self.waypoints[self.i] if self.i < 3 else self.currentWP

        cwp_x, cwp_y, cwp_z = curr_waypt
        cp_x, cp_y, cp_z = currpos
        
        euclidean_distance = math.sqrt((cp_x - cwp_x) ** 2 + (cp_y - cwp_y) ** 2 + (cp_z - cwp_z) ** 2)

        if euclidean_distance < 0.1 and not self.landed and self.i < 3:
            curr_img = self.fetchLatestImage()                      # Get image
            gray_img = cv2.cvtColor(curr_img, cv2.COLOR_BGR2GRAY)   # convert to grayscale

            detector = apriltag.Detector()
            result = detector.detect(gray_img)

            if result[0].tag_id == 4:
                self.landed = True
                self.t = time

            else:
                if self.i < 3:
                    self.i += 1
        else:
            xyz_desired = curr_waypt

        if self.landed:

            if time < self.t + 5:
                xyz_desired = [cp_x, cp_y, 0.5]
            
            elif time >= self.t + 5:
                self.landed = False
                xyz_desired = [cp_x, cp_y, 1.5]
                self.i += 1
                
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
