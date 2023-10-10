from djitellopy import Tello
import pandas as pd
import math
import time


df = pd.read_csv('/home/pear/Downloads/testtraj.csv')

tello = Tello()

tello.connect()
tello.takeoff()

z = tello.get_height()

init_x = 0
init_y  = 0
init_z = 0


#Position based control 

for column_index in range(len(df.columns)):
    column_data = df.iloc[:, column_index]
    #print(column_data.iloc[0],column_data.iloc[1],column_data.iloc[2])
    
    #Sending incremental co-odinates to the drone
    x_loc = column_data.iloc[0] -init_x
    y_loc = column_data.iloc[1] -init_y
    z_loc = column_data.iloc[2] -init_z
    
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


    #velocity based conroller to give velocity commands to the drone
    # vx = column_data.iloc[3]
    # vy = column_data.iloc[4]
    # vz = column_data.iloc[5]
    # tello.send_rc_control(int(vy*100),int(vx*100),int(vz*100), 0)
    # time.sleep(3)



    #t = tello.get_current_state()
    #print("curr state:", t)

tello.land()
