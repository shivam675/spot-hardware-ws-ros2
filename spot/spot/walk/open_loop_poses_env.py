
#!/usr/bin/env python3
from spot.walk.helpers_helper.ik_solver import Kinematics
import numpy as np




class Poses:
    def __init__(self,):
        self.manual_control = True
        self.kinematics = Kinematics()
        self.target_value = np.inf


        self.values = {
            'base_x': (-0.02, 0.02, 0.01), 
            'base_y': (-0.007, 0.007, 0), 
            'base_z': (-0.048, 0.021, 0), 
            'roll': (-0.7853981633974483, 0.7853981633974483, 0), 
            'pitch': (-0.7853981633974483, 0.7853981633974483, 0), 
            'yaw': (-0.7853981633974483, 0.7853981633974483, 0)
            }
    
    @staticmethod
    def _evaluate_stage_coefficient(current_t, action, end_t=0.0):
        # ramp function
        p = 0.8 + action[0]
        if end_t <= current_t <= p + end_t:
            return current_t
        else:
            return 1.0


    def _signal(self, t, action, position: np.array, orientation: np.array):
        if not self.manual_control:
            stage_coeff = self._evaluate_stage_coefficient(t, action)
            staged_value = self.target_value * stage_coeff
            self.values[self.next_pose] = (self.values[self.next_pose][0],
                                        self.values[self.next_pose][1],
                                        staged_value)
            self.position = np.array([
                self.values["base_x"][2],
                self.values["base_y"][2],
                self.values["base_z"][2]
            ])
            self.orientation = np.array([
                self.values["roll"][2],
                self.values["pitch"][2],
                self.values["yaw"][2]
            ])
        else:
            
            # self.position, self.orientation = np.array(
            #     [0, 0, 0]
            # ), np.array(
            #     [0, 0, 0]
            # )
            self.position, self.orientation = position, orientation
        
        fr_angles, fl_angles, rr_angles, rl_angles, _ = self.kinematics.solve(self.orientation, self.position)
        signal = [
            fl_angles[0], fl_angles[1], fl_angles[2],
            fr_angles[0], fr_angles[1], fr_angles[2],
            rl_angles[0], rl_angles[1], rl_angles[2],
            rr_angles[0], rr_angles[1], rr_angles[2]
        ]
        return signal