import gym

from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env

from forkbug_env import ForkbugEnv

# Instantiate the env
env = ForkbugEnv()
# Define and Train the agent
model = SAC("MlpPolicy", env, train_freq=1, gradient_steps=2, verbose=1)
model.learn(total_timesteps=10_000)

input('learning done. procced?')

obs = env.reset()
while env.step_counter < 1000:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()