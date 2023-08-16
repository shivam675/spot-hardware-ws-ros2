#!/usr/bin/env python3



import rospy
from std_msgs.msg import Int32MultiArray
import random
import math
from open_loop_walking_helper import OpenLoopWalking
import numpy as np
from scipy.interpolate import interp1d
import Time

# from 
# k = interp1d([-math.pi, math.pi], [-math.pi/2, math.pi/2])


class walker:
    def __init__(self):
        self.shoulder_inter = interp1d([-1.2, 1.2], [-1.04, 1.04])
        self.elbow_inter = interp1d([-((math.pi/2)+0.2), (math.pi/2)+0.2], [-1.04, 1.04])
        self.wrist_inter = interp1d([-0.2, math.pi+0.2], [0, 2.08])
        self.previous_angular_vals

    def get_ang_vels(self,):
        pass

    def execute(self,motor_vals: np.array):
        msg = Int32MultiArray()
        # msg.data = random.randint(0, 180)
        # print(motor_vals)

        # motor_vals -->> [fle, flw, fre, frw, rle, rlw, rre, rrw]

        motor_vals = motor_vals.tolist()
        # print(motor_vals)
        
        final_angles = []
        for idx, val in enumerate(motor_vals[0]):
            if idx in [2,8]: final_angles.append(int(math.degrees(wrist_inter(val))))
            elif idx in [5,11]: final_angles.append(180 - int(math.degrees(wrist_inter(val))))
            elif idx in [1,4,7,10] : final_angles.append(int(math.degrees(1.04 + elbow_inter(val))))
            else: final_angles.append(int(math.degrees(1.04 + shoulder_inter(val))))

        # print(final_angles)
        msg.data = final_angles
        # print(msg.data)
        publiser.publish(msg)




if __name__ == "__main__":
    rospy.init_node('servo_pub', anonymous=True)
    walker_instance = OpenLoopWalking()
    rate = rospy.Rate(10)
    publiser = rospy.Publisher('/hardware_actuator_node', Int32MultiArray, queue_size=1)
    current_time = 0
    temp_actions = np.zeros(8)

    period = rospy.get_param('period', 1/8)
    fa = rospy.get_param('fa', 0.2)
    

    # counter = 0
    while not rospy.is_shutdown():

        start_time = rospy.Time.now().to_sec()

        la = rospy.get_param('la', 0.1)
        print(la)
        actions = walker_instance.open_loop_signal(current_time, temp_actions, la)
        # print(actions)

        main(actions)
        rate.sleep()
        mid_time = rospy.Time.now().to_sec()
       
        current_time +=  - start_time + mid_time
 