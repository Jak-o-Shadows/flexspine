import os
import time

import numpy as np

import gym
import pybullet as p

import mine



def main():
    print("create env")
    
    env = mine.MyGym("robot2.urdf")
    env.reset()
    
    while 1:
        time.sleep(0.02)
        a = [0]*13
        obs, r, done, _ = env.step(a)
        


if __name__ == "__main__":
    main()