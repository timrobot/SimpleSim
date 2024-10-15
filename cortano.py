### Note! This is just a virtual wrapper around the environment to make sure that your code is working.
### If you want to make sure that your code works on the real robot, copy over your source code files to the robot.

from sim3d import AprilcubeClawbotEnv
import lan

color = None
depth = None
pose = (0, 0, 0)
env = None

class RealsenseCamera:
  def __init__(self):
    self.w = 640
    self.h = 360
    self.fx = 331.4
    self.fy = 331.4
    self.cx = 320
    self.cy = 180
    self.depth_scale = 0.001

  def read(self):
    global color, depth, env
    if env._hasread:
      env._hasread = False
    else:
      action = [0] * 10
      env.obs, rew, term, info = env.step(action)
      color = info["color"]
      depth = info["depth"]
    return color, depth
  
class VexJoystick:
  def __init__(self, keys):
    self.keys = keys

class VexV5(AprilcubeClawbotEnv):
  def __init__(self, render=True):
    global env
    if env is not None:
      return
    else:
      env = self

    super().__init__()
    if render:
      self.render()
    self.motors = [0] * 10
    self.obs, info = self.reset()

    global color, depth
    color = info["color"]
    depth = info["depth"]
    self._hasread = True

  def read(self):
    motors = [x / 100. for x in self.motors]
    action = [motors[0], motors[2], motors[7], 0, 0, 0, 0, 0, 0, -motors[9]]
    self.obs, rew, term, info = self.step(action)
    sensors = [
      0, action[0], 0,
      0, action[9], 0,
      self.obs[3], self.obs[4], 0,
      0, action[2], self.obs[-1]
    ]

    global color, depth
    color = info["color"]
    depth = info["depth"]

    self._hasread = True
    return sensors, 100
  
  @property
  def controller(self):
    return VexJoystick(super().keys)
  
  def running(self):
    r = super().running()
    global color, depth, env
    if self._hasread:
      self._hasread = False
    else:
      motors = [x / 100. for x in self.motors]
      action = [motors[0], motors[2], motors[7], 0, 0, 0, 0, 0, 0, -motors[9]]
      self.obs, _, __, ___ = self.step(action)
      print(super().keys)
    return r