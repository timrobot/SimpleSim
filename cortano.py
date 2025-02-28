### Note! This is just a virtual wrapper around the environment to make sure that your code is working.
### If you want to make sure that your code works on the real robot, copy over your source code files to the robot.

from environments import CanClawbotEnv, MultiplayerEnv
import numpy as np

pose = (0, 0, 0)
env = None

_camera_read_active = False
_sensor_read_active = False
_running_in_play = False

_color = None
_depth = None
_obs = None
_rew = 0.0
_term = False
_info = None
_motors = None

def clawbot_control():
  global _motors
  motors = [x / 100. for x in _motors]
  action = [motors[0], 0, motors[2], 0, 0, 0, 0, motors[7], 0, -motors[9]]
  global _obs, _rew, _term, _info
  _obs, _rew, _term, _info = env.step(action)
  return action

class RealsenseCamera:
  def __init__(self):
    self.w = 640
    self.h = 360
    self.fx = 331.4
    self.fy = 331.4
    self.cx = 320
    self.cy = 180
    self.depth_scale = 0.001

  def __del__(self):
    global env
    env.stop()

  def read(self):
    global _color, _depth, env, _camera_read_active, _sensor_read_active
    if not _camera_read_active:
      _camera_read_active = True
    elif not _sensor_read_active:
      global _motors
      if not env:
        env = CanClawboxEnv(autolaunch=True) # we dont care about multiplayer here
        env.render()
        _motors = [0] * env.action_space.shape[0]
      clawbot_control()
      _color = _info['color']
      _depth = _info['depth']
    return _color, _depth
  
class VexController:
  def __init__(self, keys):
    self.keys = keys

class VexV5:
  def __init__(self, envname='MultiplayerEnv', render=True):
    global env
    if env is None:
      if envname == 'CanClawbotEnv':
        env = CanClawbotEnv(autolaunch=True)
      elif envname == 'MultiplayerEnv':
        env = MultiplayerEnv(autolaunch=True)

    self.render = render

    global _motors
    _motors = [0] * env.action_space.shape[0]
    clawbot_control()
    if render:
      env.render()
      global _color, _depth, _info
      if _color is None:
        _color = _info['color']
        _depth = _info['depth']

  def __del__(self):
    global env
    env.stop()
    
  def read(self):
    """Return sensor values and battery life

    Returns:
        sensors: list of sensors
        battery: battery life
    """
    action = clawbot_control()
    global _obs
    sensors = [
      0, action[0], 0,
      0, action[9], 0,
      np.degrees(_obs[3]), _obs[4], 0,
      np.degrees(_obs[10]), action[2], _obs[9]
    ]

    if self.render:
      global _color, _depth, _info
      _color = _info['color']
      _depth = _info['depth']

    global _sensor_read_active
    _sensor_read_active = True
    return sensors, 100
  
  @property
  def controller(self):
    return VexController(env.keys)
  
  @property
  def motors(self):
    global _motors
    return _motors
  
  def running(self):
    """Return whether or not robot is up and running.

    Returns:
        _type_: _description_
    """
    global env, _camera_read_active, _sensor_read_active, _running_in_play
    r = env.running()
    if not _running_in_play:
      _running_in_play = True
    elif not _camera_read_active and not _sensor_read_active:
      clawbot_control()
      if self.render:
        global _color, _depth, _info
        _color = _info['color']
        _depth = _info['depth']
    return r
  
  def reset(self):
    """Hidden function for simulation only. Do not use on real robot.

    Returns:
        _type_: _description_
    """
    return env.reset()
  
  @property
  def obs(self):
    return _obs
  @property
  def rew(self):
    return _rew
  @property
  def term(self):
    return _term
  @property
  def info(self):
    return _info