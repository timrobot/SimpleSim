import cv2
import numpy as np
import sys
import platform
if platform.system() == "Darwin":
  import customtkinter as ctk
else:
  import pygame
from multiprocessing import (
  Process,
  Lock,
  Array,
  RawArray,
  Value
)
from ctypes import c_uint8, c_float
from PIL import Image, ImageTk
import logging
import lan
import time
import webbrowser
from collections import namedtuple
import mujoco
import pyrr

def _depth2rgb(depth):
  return cv2.applyColorMap(np.clip(np.sqrt(depth) * 4, 0, 255).astype(np.uint8), cv2.COLORMAP_HSV)

def _ctk_interface(key_values, color_buf, depth_buf):
  ctk.set_appearance_mode("Dark")
  ctk.set_default_color_theme("blue")
  app = ctk.CTk()
  app.geometry("1340x400")

  h, w = lan.frame_shape
  color_np = cv2.cvtColor(np.frombuffer(color_buf, np.uint8).reshape((h, w, 3)), cv2.COLOR_RGB2BGR)
  depth_np = np.frombuffer(depth_buf, np.uint16).reshape((h, w))

  depth_image = Image.fromarray(_depth2rgb(depth_np))
  color_image = Image.fromarray(color_np)
  color_photo = ImageTk.PhotoImage(image=color_image)
  depth_photo = ImageTk.PhotoImage(image=depth_image)

  color_canvas = ctk.CTkCanvas(app, width=lan.frame_shape[1], height=lan.frame_shape[0])
  color_canvas.place(x=20, y=20)
  canvas_color_object = color_canvas.create_image(0, 0, anchor=ctk.NW, image=color_photo)
  depth_canvas = ctk.CTkCanvas(app, width=lan.frame_shape[1], height=lan.frame_shape[0])
  depth_canvas.place(x=680, y=20)
  canvas_depth_object = depth_canvas.create_image(0, 0, anchor=ctk.NW, image=depth_photo)

  def animate():
    app.after(8, animate)

    depth_image = Image.fromarray(_depth2rgb(depth_np))
    color_np = cv2.cvtColor(np.frombuffer(color_buf, np.uint8).reshape((h, w, 3)), cv2.COLOR_RGB2BGR)
    color_image = Image.fromarray(color_np)

    color_photo.paste(color_image)
    depth_photo.paste(depth_image)

    color_canvas.itemconfig(canvas_color_object, image=color_photo)
    depth_canvas.itemconfig(canvas_depth_object, image=depth_photo)

  keycodes = {}
  keyrelease = {}

  def on_key_press(event):
    charcode = ord(event.char) if event.char else None
    if charcode and charcode > 0 and charcode < 128:
      keycodes[event.keycode] = charcode
      keyrelease[event.keycode] = time.time()
      key_values[charcode] = 1

  def on_key_release(event):
    charcode = None
    if event.keycode in keycodes: charcode = keycodes[event.keycode]
    if charcode and charcode > 0 and charcode < 128:
      def release_check():
        if time.time() - keyrelease[event.keycode] > 0.099:
          key_values[charcode] = 0
      keyrelease[event.keycode] = time.time()
      app.after(100, release_check)

  app.bind("<KeyPress>", on_key_press)
  app.bind("<KeyRelease>", on_key_release)
  app.after(8, animate)
  app.mainloop()
  lan.stop()
  sys.exit(0)

def _pygame_interface(key_values, color_buf, depth_buf,
    gamepad_axes=None, axes_len=None, gamepad_btns=None, btns_len=None, gamepad_hats=None, hats_len=None):
  pygame.init()
  screen = pygame.display.set_mode((1340, 400))
  clock = pygame.time.Clock()

  h, w = lan.frame_shape
  color_np = np.frombuffer(color_buf, np.uint8).reshape((h, w, 3))
  depth_np = np.frombuffer(depth_buf, np.uint16).reshape((h, w))

  gamepad = None
  if pygame.joystick.get_count() > 0:
    gamepad = pygame.joystick.Joystick(0)
    if btns_len is not None:
      btns_len.value = gamepad.get_numbuttons()
    if axes_len is not None:
      axes_len.value = gamepad.get_numaxes()
    if hats_len is not None:
      hats_len.value = gamepad.get_numhats() * 2

  running = True
  while running:
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        pygame.quit()
        running = False
        lan.stop()
        sys.exit(0)
      elif event.type == pygame.KEYDOWN:
        for i in range(26):
          charcode = chr(ord('a') + i)
          if event.key == getattr(pygame, f"K_{charcode}"):
            key_values[ord(charcode)] = 1
      elif event.type == pygame.KEYUP:
        for i in range(26):
          charcode = chr(ord('a') + i)
          if event.key == getattr(pygame, f"K_{charcode}"):
            key_values[ord(charcode)] = 0

    if gamepad is not None and gamepad_axes is not None and gamepad_btns is not None:
      gamepad_axes[:axes_len.value] = [gamepad.get_axis(i) for i in range(axes_len.value)]
      gamepad_btns[:btns_len.value] = [gamepad.get_button(i) for i in range(btns_len.value)]
      hatvs = []
      for i in range(hats_len.value // 2):
        hatvs += list(gamepad.get_hat(i))
      gamepad_hats[:hats_len.value] = hatvs

    screen.fill(pygame.Color("#1E1E1E"))

    color_surf = pygame.image.frombuffer(color_np.tobytes(), (w, h), "BGR")
    depth_surf = pygame.image.frombuffer(_depth2rgb(depth_np).tobytes(), (w, h), "RGB")

    screen.blit(color_surf, (20, 20))
    screen.blit(depth_surf, (680, 20))

    pygame.display.update()
    clock.tick(60)

  lan.stop()
  sys.exit(0)


def _bug_fix_angles(qpos):
  """Fix angles to be in the range [-pi, pi]."""
  for i in range(len(qpos)):
    qpos[i] = np.mod(qpos[i] + np.pi, 2 * np.pi) - np.pi
  return qpos


class ThreeMujocoEnv:
  def __init__(self, mujoco_model_path, htmlpath, port=9999, httpport=8765):
    """Remote Interface showing the data coming in from the robot

    Args:
        host (str, optional): host ip of the robot. Defaults to "0.0.0.0".
    """
    self.htmlpath = htmlpath
    self.port = port
    self.httpport = httpport
    
    with open(mujoco_model_path, 'r') as fp:
      model_xml = fp.read()
    self.model = mujoco.MjModel.from_xml_string(model_xml)
    self.model_data = mujoco.MjData(self.model)
    self.time_duration = 0.05

    self.sensor_names = [self.model.sensor_adr[i] for i in range(self.model.nsensor)]
    self.actuator_names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(self.model.nu)]
    self.body_names = self.model.names.decode('utf-8').split('\x00')[1:]

    self.keyboard_buf = RawArray(c_uint8, 128)
    self.ui_task = None

    # OpenAI Gym convenience fields
    self._steps = 0
    self.max_steps = 1000
    self.observation_space = namedtuple('Box', ['high', 'low', 'shape'])
    self.observation_space.shape = (self.model.nsensor,)
    self.observation_space.low = np.full(self.observation_space.shape, float('-inf')).tolist()
    self.observation_space.high = np.full(self.observation_space.shape, float('inf')).tolist()
    self.action_space = namedtuple('Box', ['high', 'low', 'shape'])
    self.action_space.shape = (self.model.nu,)
    self.action_space.low  = self.model.actuator_ctrlrange[:,0].tolist()
    self.action_space.high = self.model.actuator_ctrlrange[:,1].tolist()

    self.btns = RawArray(c_uint8, 32)
    self.axes = RawArray(c_float, 24)
    self.hats = RawArray(c_float, 16)
    self.btnslen = Value(c_uint8, 0)
    self.axeslen = Value(c_uint8, 0)
    self.hatslen = Value(c_uint8, 0)

    self._mj2three_transform = np.array([
      [ 1, 0, 0, 0],
      [ 0, 0, 1, 0],
      [ 0,-1, 0, 0],
      [ 0, 0, 0, 1],
    ])

  def __del__(self):
    self.stop()

  def stop(self):
    lan.stop()
    if self.ui_task:
      self.ui_task.kill()
    sys.exit(1)

  @property
  def keys(self):
    return {
      chr(x): self.keyboard_buf[x] for x in range(128)
    }

  @property
  def joybtn(self):
    return np.frombuffer(self.btns, np.uint8)[:self.btnslen.value]

  @property
  def joyaxis(self):
    return np.frombuffer(self.axes, np.float32)[:self.axeslen.value]
  
  @property
  def joyhat(self):
    return np.frombuffer(self.hats, np.float32)[:self.hatslen.value]
  
  def running(self):
    # for convenience purposes only now, and to check ui teardown
    if not lan.running():
      lan.stop()
      if self.ui_task:
        self.ui_task.kill()
        self.ui_task = None
    return True
  
  def reset(self):
    """
    Convenience function to act like OpenAI Gym reset()
    """
    self._steps = 0
    info = namedtuple('EnvInfo', ['time'])
    info.time = 0

    mujoco.mj_resetData(self.model, self.model_data)
    for i in range(self.model.njnt):
      self.model_data.qpos[i] = 0
      self.model_data.qvel[i] = 0
    self.on_reset()
    _bug_fix_angles(self.model_data.qpos)
    mujoco.mj_forward(self.model, self.model_data)
    _bug_fix_angles(self.model_data.qpos)
    sensor_values = self.model_data.sensordata.copy()
    observation = self.make_observation(sensor_values)

    return observation, info
  
  def _transform_mj2three(self, body_name):
    body_adr = self.body_names.index(body_name)
    T = np.eye(4)
    T[:3,:3] = self.model_data.xmat[body_adr].reshape((3, 3))
    T[:3, 3] = self.model_data.xpos[body_adr]
    T = self._mj2three_transform @ T @ self._mj2three_transform.T
    q = pyrr.Quaternion.from_matrix(T[:3,:3])
    t = T[:3,3]
    return {
      'name': body_name,
      'position': t,
      'quaternion': q # format: xyzw, same as three.js
    }
  
  def step(self, action):
    """
    Convenience function to act like OpenAI Gym step(), since setting motor values does
    not actually write motor values due to the Queue command system in simulation
    """
    assert(len(action) == self.action_space.shape[0])
    self._steps += 1

    info = namedtuple('EnvInfo', ['time'])
    info.time = self._steps * self.time_duration

    # use mujoco physx in order to step()
    action_range = np.array(self.action_space.high) - np.array(self.action_space.low)
    action = (np.array(action) - np.array(self.action_space.low)) / action_range
    actuator_low  = self.model.actuator_ctrlrange[:,0]
    actuator_high = self.model.actuator_ctrlrange[:,1]
    action = action * (actuator_high - actuator_low) + actuator_low
    for i in range(self.action_space.shape[0]):
      self.model_data.ctrl[i] = action[i]

    t = self.time_duration
    while t - self.model.opt.timestep > 0:
      t -= self.model.opt.timestep
      _bug_fix_angles(self.model_data.qpos)
      mujoco.mj_step(self.model, self.model_data)
      _bug_fix_angles(self.model_data.qpos)
    sensor_values = self.model_data.sensordata.copy()
    observation = self.make_observation(sensor_values)
    reward_value = self.calc_reward(action, observation)
    terminal_value = self.is_terminal(action, observation)

    return observation, reward_value, terminal_value, info
  
  def render(self, autolaunch=True):
    """Actually renders the camera frames, which may be necessary for some algorithms
    """
    if not lan.running():
      lan.start(self.htmlpath, self.port, self.httpport) # for visualization only
      time.sleep(2.0)
      if autolaunch:
        print("launching web browser")
        self.browser_process = webbrowser.open(f"http://127.0.0.1:{self.httpport}")

    if not self.ui_task:
      while lan.color_buf is None:
        continue
      if platform.system() == "Darwin":
        self.ui_task = Process(target=_ctk_interface, args=(self.keyboard_buf, lan.color_buf, lan.depth_buf))
      else:
        self.ui_task = Process(target=_pygame_interface, args=(
          self.keyboard_buf, lan.color_buf, lan.depth_buf,
          self.axes, self.axeslen, self.btns, self.btnslen, self.hats, self.hatslen))
      self.ui_task.start()

    # the order of the joints matters! make sure the joints are in the correct order in both xml and html
    return lan.render(self.on_render())

  def on_reset(self):
    pass

  def on_render(self):
    return []

  def make_observation(self, sensor_values):
    # abstract placeholder
    return sensor_values

  def calc_reward(self, action, obs):
    # abstract placeholder
    return 0
  
  def is_terminal(self, action, obs):
    # abstract placeholder
    return False

class PendulumEnv(ThreeMujocoEnv):
  def __init__(self, port=9999, httpport=8765):
    super(PendulumEnv, self).__init__('./env/pendulum.xml', './env/pendulum.html', port, httpport)
    self.observation_space.shape = (3,)
    self.observation_space.low = [-1., -1., float('-inf')]
    self.observation_space.high = [-1., -1., float('inf')]
    self.action_space.low = [-1.]
    self.action_space.high = [1.]

  def on_reset(self):
    self.model_data.qpos[0] = np.random.uniform(-np.pi, np.pi)
    self.model_data.qvel[0] = 0

  def on_render(self):
    return [
      self._transform_mj2three('pendulum_joint')
    ]

  def make_observation(self, sensor_values):
    return [np.cos(sensor_values[0] + np.pi), np.sin(sensor_values[0] + np.pi), sensor_values[1]]
  
  def calc_reward(self, action, obs):
    angular_distance = np.arccos(obs[0])
    return -np.abs(angular_distance) + np.exp(-angular_distance**2 / 2) # this combined differential reward is better than just -np.abs(diff)
  
  def is_terminal(self, action, obs):
    return False
  
class CanClawbotEnv(ThreeMujocoEnv):
  def __init__(self, port=9999, httpport=8765, autolaunch=True):
    self.observation_space.shape = (11,)
    self.observation_space.low = [-np.inf] * observation_space.shape[0]
    self.observation_space.high = [np.inf] * observation_space.shape[0]
    action_space = namedtuple('Box', ['high', 'low', 'shape'])
    action_space.shape = (10,)
    action_space.low = [-1.0] * action_space.shape[0]
    action_space.high = [1.0] * action_space.shape[0]
    super(CanClawbotEnv, self).__init__('./env-can-clawbot.html', observation_space, action_space, port, httpport, autolaunch)

# class MultiplayerEnv(ThreeMujocoEnv):
#   def __init__(self, port=9999, httpport=8765, autolaunch=True):
#     observation_space = namedtuple('Box', ['high', 'low', 'shape'])
#     observation_space.shape = (11,)
#     observation_space.low = [-np.inf] * observation_space.shape[0]
#     observation_space.high = [np.inf] * observation_space.shape[0]
#     action_space = namedtuple('Box', ['high', 'low', 'shape'])
#     action_space.shape = (10,)
#     action_space.low = [-1.0] * action_space.shape[0]
#     action_space.high = [1.0] * action_space.shape[0]
#     super(MultiplayerEnv, self).__init__('./env-multiplayer.html', observation_space, action_space, port, httpport, autolaunch)

# import mujoco.viewer
# viewer = None
# def render(model, data):
#   global viewer
#   """Render the environment."""
#   if viewer is None:
#     viewer = mujoco.viewer.launch_passive(model, data)
#   if viewer.is_running():
#     viewer.sync()
#   else:
#     viewer.close()
#     viewer = mujoco.viewer.launch_passive(model, data)

if __name__ == "__main__":
  env = PendulumEnv()
  env.reset()
  env.render()
  while env.running():
    next_obs, reward, term, info = env.step([0])
    env.render()
