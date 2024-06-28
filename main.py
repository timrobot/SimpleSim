from sim3d import TennisBallClawbotEnv

if __name__ == "__main__":
  env = TennisBallClawbotEnv()
  env.render()
  obs = env.reset()
  while env.running():
    y = env.keys["w"] - env.keys["s"]
    x = env.keys["d"] - env.keys["a"]
    action = [0] * 10
    action[0] = y + x
    action[9] = -y + x
    action[1] = env.keys["p"] - env.keys["l"]
    action[2] = env.keys["o"] - env.keys["k"]
    next_obs, reward, term, _ = env.step(action)