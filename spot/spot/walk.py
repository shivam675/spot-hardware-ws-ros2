#!/usr/bin/env python3

from spot.walk.open_loop_walking_helper import OpenLoopWalking
import numpy as np
from scipy.interpolate import interp1d
import math
import time
from adafruit_servokit import ServoKit


class StartWalking:
    def __init__(self,):
        self.legs = ["sim_leg_fl", "sim_leg_fr", "sim_leg_rl", "sim_leg_rr",]

        self.sim_max_angle = math.pi/2
        self.sim_min_angle = -math.pi/2

        self.irl_min_angle = 0
        self.irl_max_angle = math.pi


        self.kit = ServoKit(channels=16)

        self.irl_spot = {
            # pinout are reversed because we have attached pins in reverse order 
            # on m2 servo board
                            #[ wrist, elbow, shoulder ]
            "sim_leg_fl": [10, 9, 8],
            "sim_leg_fr": [15, 14, 13],
            "sim_leg_rl": [7, 6, 5],
            "sim_leg_rr": [2, 1, 0],
        }
        

        self.sim_spot = {

            0 : "fls", 1 : "fle", 2 : "flw",
            3 : "frs", 4 : "fre", 5 : "frw",
            6 : "rls", 7 : "rle", 8 : "rlw",
            9 : "rrs", 10 : "rre", 11 : "rrw",

        }
        # pin outs
        self.irl_spot = {

            "fls": 8, "fle": 9, "flw": 10,
            "frs": 13, "fre": 14, "frw": 15,
            "rls": 5, "rle": 6, "rlw": 7,
            "rrs": 0, "rre": 1, "rrw": 2,

        }

        base_walk_stance = [ 
                
                0.20553583, -0.92080635,  1.52989233, 
                -0.20553583, -0.92080635,  1.52989553, 
                0.20553583, -0.92080635,  1.52305466,  
                -0.20553583, -0.92080635,  1.52305884,
            
            ]
        

        
        base_stance_actions = [

            0.20553583, -0.92080635, 
            -0.20553583, -0.92080635,  
            0.20553583, -0.92080635,    
            -0.20553583, -0.92080635,  

        ]

        base_stance_sholder_value = 1.52989233


        self.shoulder_inter = interp1d([0, 3.142], [0, 180])
        self.elbow_inter = interp1d([-math.pi/2, math.pi/2], [0, 180])
        self.left_wrist_inter = interp1d([-0.2, math.pi], [0, 180])

    
    
    
    def start_walking(self, motor_values: np.array):
        motor_values = motor_values.tolist()


        final_angles = []
        for idx, val in enumerate(motor_values[0]):
            if idx in [2,8]:
                # flw, rlw
                # print(self.left_wrist_inter(val))
                self.kit.servo[self.irl_spot[self.sim_spot[idx]]].angle = int(self.left_wrist_inter(val))



            elif idx in [5,11]:
                self.kit.servo[self.irl_spot[self.sim_spot[idx]]].angle = 180 - int(self.left_wrist_inter(val))

            elif idx in [1, 7]:
                self.kit.servo[self.irl_spot[self.sim_spot[idx]]].angle = 90 + int(self.elbow_inter(val))
                # print(90 + int(self.elbow_inter(val)), int(self.elbow_inter(val)))

            elif idx in [4, 10] : 
                self.kit.servo[self.irl_spot[self.sim_spot[idx]]].angle = 90 - int(self.elbow_inter(val))
            
            elif idx in [0, 3, 6, 9]:
                final_angles.append(90)



if __name__ == '__main__':
    
    walker = StartWalking()
    walker_helper = OpenLoopWalking()
    current_time = 0
    temp_actions = np.zeros(8)

    period = 1/8
    fa = 0.8
    # fa = 1
    la = 0.05
    
    print("Walker Initiated")
    while True:
        # print('')
        start_time = time.time()
        actions = walker_helper.open_loop_signal(current_time, temp_actions, la)
        walker.start_walking(actions)
        mid_time = time.time()
        current_time +=  - start_time + mid_time
        time.sleep(0.05)

