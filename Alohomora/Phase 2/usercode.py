# import numpy as np
# import cv2
# import bpy
# import os
# import copy
# import math
# import apriltag


# class state_machine:
#     def __init__(self):
#         # User code executes every dt time in seconds
#         self.dt = 0.050
#         self.reached_0 = 0
#         self.reached_1= 0
#         self.reached_2 = 0
#         self.landing = 0
#         self.land = []

#         # location of each tag
#         self.waypoints = [
#             [-1.7,      0.8,    1.5],
#             [-1.2,     -1.6,      1.5],
#             [0.9,     0.7,    1.5]
#         ]

#         # Current waypoint
#         self.currentWP = [0,    -0.2,    1.5]

#     def euc_dist(self,currpos,wayp):
#         return(math.sqrt((currpos[0]-wayp[0])**2+(currpos[1]-wayp[1])**2+(currpos[2]-wayp[2])**2))
    
#     def exec_landing(self,currpos):
#         return(currpos[0],currpos[1],0.1)
    
#     def scan_apriltag(self,time,currpos):
#         image = self.fetchLatestImage()
#         img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#         #img = cv2.imread(image.cv2.IMREAD_GRAYSCALE)            
#         detector = apriltag.Detector()
#         result = detector.detect(img)
#         print(result[0].tag_id)
#         xyz = []
#         if result[0].tag_id == 4:
#             self.landing = 1
#             a = time
#             xyz = self.exec_landing(currpos)
        
#         return xyz
        



#     def step(self, time, currpos):
#         """
#         Input: time, current position in blender frame (x, y, z)
#         Output: desired position
#             Si unit unless specified otherwise

#             FILL YOUR CODE HERE!
#         """

#         # Write your logic here. Do not edit anything else apart from paths.

#         if self.reached_0 == 0:
#             xyz_desired = self.waypoints[0]
        

#         if(self.euc_dist(currpos,self.waypoints[0])) < 0.1 and self.reached_0 == 0 and self.landing == 0 :
#             self.reached_0 = 1
#             self.land = self.scan_apriltag(time,currpos)
#         elif self.reached_0 ==1 and self.reached_1 == 0 and self.reached_2 == 0 and self.landing == 0:
#             xyz_desired = self.waypoints[1]
        
        
#         if(self.euc_dist(currpos,self.waypoints[1])) < 0.1 and self.reached_0 == 1 and self.reached_1 == 0 and self.landing == 0:
#             self.reached_1 = 1
#             self.land = self.scan_apriltag(time,currpos)
#         elif self.reached_0 ==  1 and self.reached_1 == 1 and self.landing == 0:    
#             xyz_desired = self.waypoints[2]


#         if(self.euc_dist(currpos,self.waypoints[2])) < 0.1 and self.reached_2 == 0 and self.landing == 0:
#             self.reached_2 = 1
#             self.land = self.scan_apriltag(time,currpos)

#         if self.landing == 1 :
#              xyz_desired = self.land
#              if(self.euc_dist(currpos,self.land)) < 0.1:
#                  self.landing = 0
#                  xyz_desired = [self.land[0], self.land[1], 1.5]


#         return xyz_desired
        

#     def fetchLatestImage(self):
#         # Fetch image - renders the camera, saves the rendered image to a file and reads from it. 
#         path_dir = bpy.data.scenes["Scene"].node_tree.nodes["File Output"].base_path

#         # Render Drone Camera
#         cam = bpy.data.objects['DownCam']    
#         bpy.context.scene.camera = cam
#         bpy.context.scene.render.filepath = os.path.join(path_dir, 'DownCam_latest.png')
#         bpy.ops.render.render(write_still=True)

#         return cv2.imread(bpy.context.scene.render.filepath)

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
       
        xyz_desired = self.currentWP
        curr_waypt = self.currentWP
        if self.i < 3:
            curr_waypt = self.waypoints[self.i]
        
        cwp_x, cwp_y, cwp_z, = curr_waypt[0], curr_waypt[1], curr_waypt[2]
        cp_x, cp_y, cp_z, = currpos[0], currpos[1], currpos[2]
        euc_dis = math.sqrt(((cp_x - cwp_x)**2) + ((cp_y - cwp_y)**2) + ((cp_z - cwp_z)**2))

        if euc_dis<0.1 and self.landed == False and self.i<3:
            curr_img = self.fetchLatestImage()                        #Get image 
            gray_img = cv2.cvtColor(curr_img, cv2.COLOR_BGR2GRAY)   #convert to grayscale
            detector = apriltag.Detector()  
            result = detector.detect(gray_img)
            if result[0].tag_id == 4: 
                self.landed = True 
                self.t = time
            else:
                if self.i<3: 
                    self.i = self.i+1



        else:
            xyz_desired = curr_waypt

        if self.landed: 
            if time< self.t+5:
                xyz_desired = [cp_x,cp_y,0.5]
            if time > self.t+5:
                self.landed = False
                xyz_desired = [cp_x,cp_y,1.5]
                self.i = self.i+1

        

        print(self.i)
        return xyz_desired




        

        # if self.landed and (time - curr_time) < 5:
        #     return xyz_desired
        
        # num_waypts = len(self.waypoints)
        # # atwWP = 0
        # curr_waypt = self.waypoints[self.i]
        # xyz_desired = curr_waypt 
        # cwp_x, cwp_y, cwp_z, = curr_waypt[0], curr_waypt[1], curr_waypt[2]
        # cp_x, cp_y, cp_z, = currpos[0], currpos[1], currpos[2]

        # euc_dis = math.sqrt(((cp_x - cwp_x)**2) + ((cp_y - cwp_y)**2) + ((cp_z - cwp_z)**2))
        # threshold = 1.4
        # if euc_dis < threshold: #WAYPOINT
        #     if self.landed:
        #         self.landed = False
        #         return [cwp_x, cwp_y, 1.5]
            
        #     #Hasn't Landed yet OR risen from landing
        #     curr_img = self.fetchLatestImage()                        #Get image 
        #     gray_img = cv2.cvtColor(curr_img, cv2.COLOR_BGR2GRAY)   #convert to grayscale
        #     detector = apriltag.Detector()  
        #     result = detector.detect(gray_img)
        #     print(result[0].tag_id)
            
            
        #     if t_num == 4:
        #         self.landed = True
        #         xyz_desired = [cwp_x, cwp_y, 0.1]
        #         curr_time = time
            
        #     self.i = self.i+1
        
        # return xyz_desired

    def fetchLatestImage(self):
        # Fetch image - renders the camera, saves the rendered image to a file and reads from it. 
        path_dir = bpy.data.scenes["Scene"].node_tree.nodes["File Output"].base_path

        # Render Drone Camera
        cam = bpy.data.objects['DownCam']    
        bpy.context.scene.camera = cam
        bpy.context.scene.render.filepath = os.path.join(path_dir, 'DownCam_latest.png')
        bpy.ops.render.render(write_still=True)

        return cv2.imread(bpy.context.scene.render.filepath)







       