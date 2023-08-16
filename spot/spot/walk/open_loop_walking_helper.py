
import math
import numpy as np



class OpenLoopWalking:

    def __init__(self, period = 1.0/8, la = 0.1, fa = 0.2) -> None:

        self.period = period
        self.goal_reached = False

        # walking init stance pose
        self.init_pose = np.array([
                                     0.15192765, -0.90412283, 1.48156545,
                                    -0.15192765, -0.90412283, 1.48156545,
                                    0.15192765, -0.90412283, 1.48156545,
                                    -0.15192765, -0.90412283, 1.48156545
                                ]),

        self.stay_still = False
        self.end_time = None
        self.la = la
        self.fa = fa


    def evaluate_brakes_stage_coeff(self, current_t, action, end_t=0.0, end_value=0.0):
        # ramp function
        p = 0.8 + action[1]
        if end_t <= current_t <= p + end_t:
            return 1 - (current_t - end_t)
        else:
            return end_value


    def evaluate_gait_stage_coeff(self, current_t, action, end_t=0.0):
        # ramp function
        # print(action)
        p = 0.8 + action[0]
        if end_t <= current_t <= p + end_t:
            return current_t
        else:
            return 1.0
    

    def open_loop_signal(self, t, action, la):

        # print(self.la)
        # la = 0.1
        # la = 0.1
        fa = 2 * la
        if self.goal_reached:
            # print('goal_reacjed')
            coeff = self.evaluate_brakes_stage_coeff(t, [0, 0], end_t=self.end_time, end_value=0.0)
            la *= coeff
            fa *= coeff
            if coeff == 0.0:
                self.stay_still = True
        start_coeff = self.evaluate_gait_stage_coeff(t, [0.0])
        # print(start_coeff)
        la *= start_coeff
        fa *= start_coeff

        l_extension = la * math.cos(2 * math.pi / self.period * t)
        f_extension = fa * math.cos(2 * math.pi / self.period * t)
        # print(self.la)
        initial_pose = self.init_pose
        # print(la)
        l_swing = - l_extension
        swing = - f_extension
        
        pose = np.array([0.0, l_extension + action[0], f_extension + action[1],
                            0.0, l_swing + action[2], swing + action[3],
                            0.0, l_swing + action[4], swing + action[5],
                            0.0, l_extension + action[6], f_extension + action[7]])
        
        # print(pose)
        action_signal = initial_pose + pose*2
        return action_signal

    


    

















