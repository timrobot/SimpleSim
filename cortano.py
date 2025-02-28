### Note! This is just a virtual wrapper around the environment to make sure that your code is working.
### If you want to make sure that your code works on the real robot, copy over your source code files to the robot.

from environments import CanClawboxEnv, MultiplayerEnv
import numpy as np

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
    if not env._camera_read_active:
      env._camera_read_active = True
    elif not env._sensor_read_active:
      motors = [x / 100. for x in env.motors]
      action = [motors[0], 0, motors[2], 0, 0, 0, 0, motors[7], 0, -motors[9]]
      env.obs, env.rew,env.term, env.info = env.gym_env.step(action)
      color = env.info["color"]
      depth = env.info["depth"]
    return color, depth
  
class VexController:
  def __init__(self, keys):
    self.keys = keys

class VexV5:
  def __init__(self, envname='MultiplayerEnv', render=True):
    global env
    if env is not None:
      return
    elif envname == 'CanClawboxEnv':
      self.gym_env = CanClawboxEnv(autolaunch=True)
    elif envname == 'MultiplayerEnv':
      self.gym_env = MultiplayerEnv(autolaunch=True)

    self.motors = [0] * 10
    self.obs, _, __, self.info = self.gym_env.step(self.motors)
    if render: self.gym_env.render()

    global color, depth
    color = self.info["color"]
    depth = self.info["depth"]
    self._camera_read_active = False
    self._sensor_read_active = False
    self._running_in_play = False
    self.obs = None
    self.rew = 0.0
    self.term = False
    self.info = None

  def read(self):
    """Return sensor values and battery life

    Returns:
        sensors: list of sensors
        battery: battery life
    """
    motors = [x / 100. for x in self.motors]
    action = [motors[0], 0, motors[2], 0, 0, 0, 0, motors[7], 0, -motors[9]]
    self.obs, self.rew, self.term, self.info = self.gym_env.step(action)
    sensors = [
      0, action[0], 0,
      0, action[9], 0,
      np.degrees(self.obs[3]), self.obs[4], 0,
      np.degrees(self.obs[11]), action[2], self.obs[10]
    ]

    global color, depth
    color = self.info["color"]
    depth = self.info["depth"]

    self._sensor_read_active = True
    return sensors, 100
  
  @property
  def controller(self):
    return VexController(self.gym_env.keys)
  
  def running(self):
    """Return whether or not robot is up and running.

    Returns:
        _type_: _description_
    """
    self._running_in_play = True
    r = self.gym_env.running()
    global color, depth, env
    if not self._running_in_play:
      self._running_in_play = True
    elif not self._camera_read_active and not self._sensor_read_active:
      motors = [x / 100. for x in self.motors]
      action = [motors[0], 0, motors[2], 0, 0, 0, 0, motors[7], 0, -motors[9]]
      self.obs, self.rew, self.term, self.info = self.gym_env.step(action)
    return r
  
  def reset(self):
    """Hidden function for simulation only. Do not use on real robot.

    Returns:
        _type_: _description_
    """
    return self.gym_env.reset()