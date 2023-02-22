from stable_baselines3.common.env_checker import check_env
from forkbug_env import ForkbugEnv

env = ForkbugEnv()
# It will check your custom environment and output additional warnings if needed
check_env(env)

obs = env.reset()
while env.step_counter < 1000:
    action = [0.5, 0, -1. + (env.step_counter / 500)]
    obs, rewards, done, info = env.step(action)
    env.render()

env.step([0., 0., 0.])
obs, _, _, _ = env.step(action)
print( obs )

input('terminate on key press')