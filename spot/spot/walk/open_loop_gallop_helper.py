#!/usr/bin/env python3

import numpy as np
from spot.walk.helpers_helper.ik_solver import Kinematics
from spot.walk.helpers_helper.gallop_gait import GaitPlanner


class IKGalloping:
    def __init__(self, ):
        self._kinematics = Kinematics()
        self._is_render = False

        self._gait_planner = GaitPlanner("gallop")
        self._base_roll = 0.3
        self._base_pitch = 0
        self._base_yaw = 0
        self.step_length = 0.7
        self.step_rotation = 0
        self.step_angle = 0
        self.step_period = 0.5
        self.goal_reached = False


    @staticmethod
    def _evaluate_stage_coefficient(current_t, end_t=0.0, width=0.001):
        # sigmoid function
        beta = p = width
        if p - beta + end_t <= current_t <= p - (beta / 2) + end_t:
            return (2 / beta ** 2) * (current_t - p + beta) ** 2
        elif p - (beta/2) + end_t <= current_t <= p + end_t:
            return 1 - (2 / beta ** 2) * (current_t - p) ** 2
        else:
            return 1
    

    @staticmethod
    def _evaluate_gait_stage_coeff(current_t, action, end_t=0.0):
        # ramp function
        p = 1. + action[1]
        if end_t <= current_t <= p + end_t:
            return current_t
        else:
            return 1.0
    
    @staticmethod
    def _evaluate_brakes_stage_coeff(current_t, action, end_t=0.0, end_value=0.0):
        # ramp function
        p = 1. + action[0]
        if end_t <= current_t <= p + end_t:
            return 1 - (current_t - end_t)
        else:
            return end_value

    def _IK_signal(self, t, action):
            
        base_pos_coeff = self._evaluate_stage_coefficient(t, width=1.5)
        gait_stage_coeff = self._evaluate_gait_stage_coeff(t, action)
        
        if self._is_render:
            position, orientation, step_length, step_rotation, step_angle, step_period = \
                self._read_inputs(base_pos_coeff, gait_stage_coeff)
        else:
            position = np.array([0.01,
                                0 * base_pos_coeff,
                                -0.007])
            orientation = np.array([self._base_roll * base_pos_coeff,
                                    self._base_pitch * base_pos_coeff,
                                    self._base_yaw * base_pos_coeff])
            step_length = (self.step_length if self.step_length is not None else 1.3) * gait_stage_coeff
            step_rotation = (self.step_rotation if self.step_rotation is not None else 0.0)
            step_angle = self.step_angle if self.step_angle is not None else 0.0
            step_period = (self.step_period if self.step_period is not None else 0.3)
        if self.goal_reached:
            brakes_coeff = self._evaluate_brakes_stage_coeff(t, action, self.end_time)
            step_length *= brakes_coeff
        frames = self._gait_planner.loop(step_length, step_angle, step_rotation, step_period, 1.0)
        fr_angles, fl_angles, rr_angles, rl_angles, _ = self._kinematics.solve(orientation, position, frames)
        signal = [
            fl_angles[0], fl_angles[1], fl_angles[2],
            fr_angles[0], fr_angles[1], fr_angles[2],
            rl_angles[0], rl_angles[1], rl_angles[2],
            rr_angles[0], rr_angles[1], rr_angles[2]
        ]
        return signal