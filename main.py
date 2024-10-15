from cortano import RealsenseCamera, VexV5

if __name__ == "__main__":
  camera = RealsenseCamera()
  robot = VexV5()

  while robot.running():
    keys = robot.controller.keys
    y = keys["w"] - keys["s"]
    x = keys["d"] - keys["a"]
    robot.motors[0] = (y + x) * 50
    robot.motors[9] = (y - x) * 50
    robot.motors[2] = (keys["p"] - keys["l"]) * 100
    robot.motors[7] = (keys["o"] - keys["k"]) * 100