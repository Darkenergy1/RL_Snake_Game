from stable_baselines3 import PPO
from tank_converted import CustomEnv


models_dir = "models/1668185148"

env = CustomEnv()
env.reset()

model_path = f"{models_dir}/10000.zip"
model = PPO.load(model_path, env=env)

episodes = 10

for ep in range(episodes):
    obs = env.reset()
    done = False
    while not done:
        action, _states = model.predict(obs)
        obs, rewards, done, info = env.step(action)
        print(rewards)