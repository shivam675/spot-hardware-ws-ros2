#!/usr/bin/env python3
from time import sleep
from adafruit_servokit import ServoKit
import random
from itertools import cycle
import math
from scipy.interpolate import interp1d

kit = ServoKit(channels=16)



class SPOT_ME:
    def __init__(self) -> None:
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

        self.base_walk_stance = [ 
                
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


        self.shoulder_inter = interp1d([0, 3.142], [0, 180])
        self.elbow_inter = interp1d([-math.pi/2, math.pi/2], [0, 180])
        self.left_wrist_inter = interp1d([-0.2, math.pi], [0, 180])



    def rectify_angle(self, idx, value):

        true_angle = self.map_angles(value)
        
        return math.degrees(true_angle)





    def set_spot_pose(self, pose: list) -> list:
        """
        Take Pose list via sim or rl model and set them to
        real spot
        """

        poses = {
            
            "sim_leg_fl" : pose[0:3],
            "sim_leg_fr" : pose[3:6],
            "sim_leg_rl" : pose[6:9],
            "sim_leg_rr" : pose[9:12],
        }

        for leg in self.legs:
            for index, motor_value in enumerate(poses[leg]):
                pin_value = self.irl_spot[leg][index]

                print(pin_value, int(self.rectify_angle(pin_value, motor_value)))
                # kit.servo[pin_value] = int(self.rectify_angle(pin_value, motor_value))
                



    def start(self, motor_values):
        
            final_angles = []
            for idx, val in enumerate(motor_values):
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


    
def main():

    stance_1 = [  0, -.5,  2, 
                    0, -1.4,  3,  
                    0, -1.4,  2.6, 
                    0, -1.4,  2.6, 
    ]
    
    spot = SPOT_ME()
    
    spot.start(spot.base_walk_stance)
    spot.start(stance_1)
    

    






if __name__ == "__main__":
    main()