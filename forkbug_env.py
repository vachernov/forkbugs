import gym
import numpy as np
from gym import spaces

import os
import pybullet as p
import pybullet_data
import random


class ForkbugEnv(gym.Env):
    """Custom Environment that follows gym interface."""

    metadata = {"render.modes": ["human"]}

    HALF_PI = 1.57079632679
    G = 9.81

    v_0 = 25.

    MAX_EPISODE_LEN = 2*100

    def __init__(self):
        # super().__init__()
        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        #self.action_space = spaces.Discrete(N_DISCRETE_ACTIONS)
        # Example for using image as input (channel-first; channel-last also works):
        #self.observation_space = spaces.Box(low=0, high=255,
                                            # shape=(N_CHANNELS, HEIGHT, WIDTH), dtype=np.uint8)

        self.step_counter = None
        p.connect(p.GUI) # or p.DIRECT for non-graphical version
        # p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40,
        #                              cameraTargetPosition=[0.55,-0.35,0.2])

        p.setAdditionalSearchPath(pybullet_data.getDataPath()) # optionally

        self.forkbugId  = None
        self.forkbugPos = None
        self.forkbugOrn = None

        self.target     = None

        self.action_space      = spaces.Box(np.array([-1]*3), np.array([1]*3)) # v, w, a
        self.observation_space = spaces.Box(np.array([-1]*6), np.array([1]*6)) # x y theta x_0 y_0 theta_0

        self.action      = None
        self.observation = None 

        self.MAX_EPISODE_LEN = 20*100

    def step(self, action):
        self.step_counter += 1
        self.__iterate_sim(action)

        self.__make_observation()

        dist2target = np.linalg.norm(self.observation[0:1] - self.observation[3:4])
        reward = float( dist2target )
        
        done = False
        if dist2target < 0.01:
            done = True
            reward = 2.
        if (self.step_counter > self.MAX_EPISODE_LEN):
            done = True
            reward = -2.

        info = {'target_position': self.target}
        return self.observation, reward, done, info

    def reset(self):
        self.step_counter = 0
        p.resetSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING ,0) # we will enable rendering after we loaded everything

        p.setGravity(0, 0, -self.G)
        self.planeId = p.loadURDF("plane.urdf")

        self.forkbugPos = [0.0, 0.0, 0.0]
        self.forkbugOrn = p.getQuaternionFromEuler([self.HALF_PI, 0, 0])
        self.forkbugId = p.loadURDF("forkbug.urdf", self.forkbugPos, self.forkbugOrn, 
                                    #useMaximalCoordinates=1, ## New feature in Pybullet
                                    flags=p.URDF_USE_INERTIA_FROM_FILE)

        
        self.target = [random.uniform(-0.6, 0.6), random.uniform(-0.6, 0.6), 0.0]

        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        self.__make_observation()
        print('[reset] dim observ: ', self.observation)
        return self.observation  #reward, done, info | can't be included

    def render(self, mode="human"):
        pass

    def close(self):
        p.disconnect()

    def __iterate_sim(self, action):
        # 
        p.setJointMotorControl2(bodyUniqueId = self.forkbugId,
                                jointIndex = 0,
                                controlMode = p.VELOCITY_CONTROL,
                                targetVelocity = self.v_0*action[0] + action[1],
                                force = 6000)

        p.setJointMotorControl2(bodyUniqueId = self.forkbugId,
                                jointIndex = 1,
                                controlMode = p.VELOCITY_CONTROL,
                                targetVelocity = self.v_0*action[0] - action[1],
                                force = 6000)

        p.setJointMotorControl2(bodyUniqueId = self.forkbugId,
                                jointIndex = 2,
                                controlMode = p.POSITION_CONTROL,
                                targetPosition = ((action[2]+1)/2)*1.003564,
                                force = 600)

        p.setJointMotorControl2(bodyUniqueId = self.forkbugId,
                                jointIndex = 3,
                                controlMode = p.POSITION_CONTROL,
                                targetPosition = ((action[2]+1)/2)*1.003564,
                                force = 600)

        # 
        p.stepSimulation()

        # 
        self.forkbugPos, self.forkbugOrn = p.getBasePositionAndOrientation(self.forkbugId)

    def __make_observation(self):
        self.forkbugPos, self.forkbugOrn = p.getBasePositionAndOrientation(self.forkbugId)

        state = [self.forkbugPos[0], self.forkbugPos[1], self.forkbugOrn[0],
                 self.target[0], self.target[1], self.target[2]] 
        self.observation = np.asarray(state).astype(np.float32)

    def __set_max_episode_len(self, value):
        self.MAX_EPISODE_LEN = value
