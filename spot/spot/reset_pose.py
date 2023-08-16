#!/usr/bin/env python3
from time import sleep
from adafruit_servokit import ServoKit
import random
from itertools import cycle
import math

kit = ServoKit(channels=16)

    
def main():

    for idx in range(16):

        if idx in [0, 5, 8, 13]:
            kit.servo[idx].angle = 90

        if idx in [1, 6, 9, 14]:
            if idx in [1, 14]:
                kit.servo[idx].angle = 0
            if idx in [6, 9]:
                kit.servo[idx].angle = 180

        if idx in [2, 7, 10, 15]:
            if idx in [2, 15]:
                kit.servo[idx].angle = 180
            if idx in [7, 10]:
                kit.servo[idx].angle = 0


        # print(idx, 90)



if __name__ == "__main__":
    main()