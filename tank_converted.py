import gym
from gym import spaces
import numpy as np
import time
from collections import deque
import cv2
import matplotlib as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation


action_history_length = 10

class CustomEnv(gym.Env):

    def __init__(self):
        super(CustomEnv, self).__init__()
        self.action_space = spaces.Discrete(5)
        self.observation_space = spaces.Box(low=-100, high=100, shape=(2+action_history_length,), dtype=np.float32)
        self.ii = 0

    def step(self, action):
        self.ii += 1
        self.prev_actions.append(action)
       
        
        # NOT PART OF TANK CODE, TAKEN FROM SNAKE CODE ONLY TO SLOW DOWN to see the print statement outputs in console for debugging
        tt_end = time.time() + 0.1
        k = -1
        while time.time() < tt_end:
            if k == -1:
                k = cv2.waitKey(1)
            else:
                continue

        Kp2_direction = action
        # Change the head position based on the button direction
        if Kp2_direction == 0:
            self.Kp2 += -40
        elif Kp2_direction == 1:
            self.Kp2 += 40
        elif Kp2_direction == 2:
            self.Kp2 += 0
        elif Kp2_direction == 3:
            self.Kp2 += 0
        elif Kp2_direction == 4:
            self.Kp2 += 0
            
            
        # i goes from 1 till the length of the time vector, last element not counted, if len(t)=1251, then you go till 1250
        if self.ii<300:
            # vol_r2[i]=vol_r2_i+3*t[i]
            self.vol_r2[self.ii] = self.vol_r2_i +  3 * self.t[self.ii]
        elif self.ii<600:
            self.vol_r2[self.ii]=self.vol_r2_i+100 - self.t[self.ii] * 3
            self.temp2=self.vol_r2[self.ii]
            self.time_temp2 = self.t[self.ii]
        elif self.ii<900:
            self.vol_r2[self.ii]=self.temp2-1 * (self.t[self.ii]-self.time_temp2)
        else:
            self.vol_r2[self.ii]=self.temp2-1 * (self.t[self.ii]-self.time_temp2) + 50
        
        
        self.error2[self.ii-1]=self.vol_r2[self.ii-1]-self.volume_Tank2[self.ii-1]
        # Compute the control inputs for all the tanks
        self.m_dot2[self.ii]=self.Kp2*self.error2[self.ii-1]
        # Compute the true tank volumes in the next time step through this numerical integration (trapezoidal rule)
        self.volume_Tank2[self.ii]=self.volume_Tank2[self.ii-1]+(self.m_dot2[self.ii-1]+self.m_dot2[self.ii])/(2*self.density_water)*(self.dt)
        
       
        
        if self.error2[self.ii-1] >= 30:
            self.reward = -100
        elif self.error2[self.ii-1] >= 20:
            self.reward = -10
        elif self.error2[self.ii-1] >= 10:
            self.reward = -5
        elif self.error2[self.ii-1] >= 5:
            self.reward = -1
        elif self.error2[self.ii-1] >= 2:
            self.reward = -1
        elif 2 > self.error2[self.ii-1] > -2:
            self.reward = 200
        elif self.error2[self.ii-1] <= -30:
            self.reward = -100
        elif self.error2[self.ii-1] <= -20:
            self. reward = -10
        elif self.error2[self.ii-1] <= -10:
            self.reward = -5
        elif self.error2[self.ii-1] <= -5:
            self.reward = -1 
        elif self.error2[self.ii-1] <= -2:
            self.reward = -1
          
        
        
            
        
        if self.ii >= 1250:
            self.done = True
        else:
            self.done = False
        
        info = {}
 
        observation = [self.vol_r2[self.ii], self.volume_Tank2[self.ii]] + list(self.prev_actions)
        observation = np.array(observation)
        
        
        print(f'Iter: {self.ii} | Vr: {self.vol_r2[self.ii]:.2f} | Vc: {self.volume_Tank2[self.ii]:.2f} | Kp2: {self.Kp2:.2f} | Error: {self.error2[self.ii-1]:.2f} | Reward: {self.reward:.2f}')

        
        return observation, self.reward, self.done, info


    def reset(self):
        
        self.radius=5 # Radius of the tank - The tank is round.
        self.bottom=0 # Initial volume of the tank
        self.final_volume=100 # Final volume of the tank
        self.dVol=10 # The change of volume on the vertical scale.
        self.width_ratio=1 #Necessary for the horizontal axis
        self.dt=0.04 # Time interval
        self.t0=0 # Initial time of the simulation
        self.t_end=50 # Final time of the simulation
        self.frame_amount=int(self.t_end/self.dt) # Frame amount of the simulation
        self.t=np.arange(self.t0,self.t_end+self.dt,self.dt) # Time vector
        self.len_t = len(self.t)
        self.density_water=1000  # [kg/m^3]
        self.Kp2=0 # Proportional constant for the 2nd tank
        self.vol_o2_i=40 # Initial volume of the 2nd tank
        self.vol_r2_i=5 # Initial reference volume of the 2nd tank
        self.vol_r2=np.zeros(len(self.t)) # 0 vector for storing reference volume values
        self.vol_r2[0]=self.vol_r2_i
        self.volume_Tank2=np.zeros(len(self.t)) # 0 vector for true volume values
        self.volume_Tank2[0]=self.vol_o2_i # Insert the initial true volume as the initial element of the vector
        self.error2=np.zeros(len(self.t)) # Create a 0 vector to store errors in the simulation
        self.m_dot2=self.Kp2*self.error2 # Compute a 0 vector to store massflow control inputs
        self.Kp2_direction = 4
        self.ii=0
        
        
        self.done = False
        self.prev_actions = deque(maxlen = action_history_length)  # however long we aspire the snake to be
        for i in range(action_history_length):
            self.prev_actions.append(-1) # to create history
        
        # create observation:
        #observation = [self.vol_r2[i], self.volume_Tank2[i]] + list(self.prev_actions)
        observation = [5, 40] + list(self.prev_actions)
        observation = np.array(observation)
        
        return observation

        























