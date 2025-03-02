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

def _depth2rgb(depth):
  return cv2.applyColorMap(np.clip(np.sqrt(depth) * 4, 0, 255).astype(np.uint8), cv2.COLORMAP_HSV)

def _ctk_interface(key_values, color_buf, depth_buf, color2_buf):
  ctk.set_appearance_mode("Dark")
  ctk.set_default_color_theme("blue")
  app = ctk.CTk()
  app.geometry("1600x900")

  h, w = lan.frame_shape
  color_np = cv2.cvtColor(np.frombuffer(color_buf, np.uint8).reshape((h, w, 3)), cv2.COLOR_RGB2BGR)
  depth_np = np.frombuffer(depth_buf, np.uint16).reshape((h, w))
  color2_np = np.frombuffer(color2_buf, np.uint8).reshape((h, w, 3))

  depth_image = Image.fromarray(_depth2rgb(depth_np))
  color_image = Image.fromarray(color_np)
  color2_image = Image.fromarray(color2_np)
  color_photo = ImageTk.PhotoImage(image=color_image)
  depth_photo = ImageTk.PhotoImage(image=depth_image)
  color2_photo = ImageTk.PhotoImage(image=color2_image)

  color_canvas = ctk.CTkCanvas(app, width=lan.frame_shape[1], height=lan.frame_shape[0])
  color_canvas.place(x=20, y=20)
  canvas_color_object = color_canvas.create_image(0, 0, anchor=ctk.NW, image=color_photo)
  depth_canvas = ctk.CTkCanvas(app, width=lan.frame_shape[1], height=lan.frame_shape[0])
  depth_canvas.place(x=680, y=20)
  canvas_depth_object = depth_canvas.create_image(0, 0, anchor=ctk.NW, image=depth_photo)
  color2_canvas = ctk.CTkCanvas(app, width=lan.frame_shape[1], height=lan.frame_shape[0])
  color2_canvas.place(x=20, y=400)
  canvas_color2_object = color2_canvas.create_image(0, 0, anchor=ctk.NW, image=color2_photo)

  def animate():
    app.after(8, animate)

    depth_image = Image.fromarray(_depth2rgb(depth_np))
    color_np = cv2.cvtColor(np.frombuffer(color_buf, np.uint8).reshape((h, w, 3)), cv2.COLOR_RGB2BGR)
    color_image = Image.fromarray(color_np)
    c2 = lan.cam2_enable.value
    if c2:
      color2_image = Image.frombuffer('RGB', (w, h), color_buf, 'raw')

    color_photo.paste(color_image)
    depth_photo.paste(depth_image)
    if c2:
      color2_photo.paste(color2_image)

    color_canvas.itemconfig(canvas_color_object, image=color_photo)
    depth_canvas.itemconfig(canvas_depth_object, image=depth_photo)
    if c2:
      color2_canvas.itemconfig(canvas_color2_object, image=color2_photo)

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

def _pygame_interface(key_values, color_buf, depth_buf, color2_buf,
    gamepad_axes=None, axes_len=None, gamepad_btns=None, btns_len=None, gamepad_hats=None, hats_len=None):
  pygame.init()
  screen = pygame.display.set_mode((1600, 900))
  clock = pygame.time.Clock()

  h, w = lan.frame_shape
  color_np = np.frombuffer(color_buf, np.uint8).reshape((h, w, 3))
  depth_np = np.frombuffer(depth_buf, np.uint16).reshape((h, w))
  color2_np = np.frombuffer(color2_buf, np.uint8).reshape((h, w, 3))

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
        running = True
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
    color2_surf = pygame.image.frombuffer(color2_np.tobytes(), (w, h), "BGR")

    screen.blit(color_surf, (20, 20))
    screen.blit(depth_surf, (680, 20))
    screen.blit(color2_surf, (20, 400))

    pygame.display.update()
    clock.tick(60)
  lan.stop()
  sys.exit(0)

class ThreeSimEnv:
  def __init__(self, htmlpath=None, observation_space=None, action_space=None, port=9999, httpport=8765, autolaunch=True):
    """Remote Interface showing the data coming in from the robot

    Args:
        host (str, optional): host ip of the robot. Defaults to "0.0.0.0".
    """
    lan.start(htmlpath, port, httpport)
    self.keyboard_buf = RawArray(c_uint8, 128)
    time.sleep(2.0)
    if autolaunch:
      self.browser_process = webbrowser.open(f"http://127.0.0.1:{httpport}")
    self.ui_task = None

    # OpenAI Gym convenience fields
    self._steps = 0
    self.max_steps = 1000
    self.observation_space = observation_space
    self.action_space = action_space

    self.btns = RawArray(c_uint8, 32)
    self.axes = RawArray(c_float, 24)
    self.hats = RawArray(c_float, 16)
    self.btnslen = Value(c_uint8, 0)
    self.axeslen = Value(c_uint8, 0)
    self.hatslen = Value(c_uint8, 0)

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
    _running = lan.running()
    if not _running:
      lan.stop()
      if self.ui_task:
        self.ui_task.kill()
        self.ui_task = None
    return _running
  
  def step(self, values):
    """
    Convenience function to act like OpenAI Gym step(), since setting motor values does
    not actually write motor values due to the Queue command system in simulation
    """
    assert(len(values) == self.action_space.shape[0])
    if not lan.running():
      raise Exception("Environment has been torn down.")
    self._steps += 1

    observation, reward, terminal, info = lan.step(values)
    terminal = terminal or self._steps >= self.max_steps
    info["time"] = self._steps * .1
    return observation, reward, terminal, info

  def reset(self, extra_info=True):
    """
    Convenience function to act like OpenAI Gym reset()
    """
    if not lan.running():
      raise Exception("Environment has been torn down.")
    self._steps = 0
    
    observation, reward, terminal, info = lan.reset()
    info["time"] = 0
    if not extra_info:
      return observation
    else:
      return observation, info
  
  def render(self, enable=True):
    lan.render(enable)
    if not self.ui_task:
      while lan.color_buf is None:
        continue
      if platform.system() == "Darwin":
        self.ui_task = Process(target=_ctk_interface, args=(self.keyboard_buf, lan.color_buf, lan.depth_buf, lan.color2_buf))
      else:
        self.ui_task = Process(target=_pygame_interface, args=(
          self.keyboard_buf, lan.color_buf, lan.depth_buf, lan.color2_buf,
          self.axes, self.axeslen, self.btns, self.btnslen, self.hats, self.hatslen))
      self.ui_task.start()
  
class CanClawbotEnv(ThreeSimEnv):
  def __init__(self, port=9999, httpport=8765, autolaunch=True):
    observation_space = namedtuple('Box', ['high', 'low', 'shape'])
    observation_space.shape = (11,)
    observation_space.low = [-np.inf] * observation_space.shape[0]
    observation_space.high = [np.inf] * observation_space.shape[0]
    action_space = namedtuple('Box', ['high', 'low', 'shape'])
    action_space.shape = (10,)
    action_space.low = [-1.0] * action_space.shape[0]
    action_space.high = [1.0] * action_space.shape[0]
    super(CanClawbotEnv, self).__init__('./env-can-clawbot.html', observation_space, action_space, port, httpport, autolaunch)
  
class PendulumEnv(ThreeSimEnv):
  def __init__(self, port=9998, httpport=8764, autolaunch=True):
    observation_space = namedtuple('Box', ['high', 'low', 'shape'])
    observation_space.shape = (3,)
    observation_space.low = [-np.inf] * observation_space.shape[0]
    observation_space.high = [np.inf] * observation_space.shape[0]
    action_space = namedtuple('Box', ['high', 'low', 'shape'])
    action_space.shape = (1,)
    action_space.low = [-1.0] * action_space.shape[0]
    action_space.high = [1.0] * action_space.shape[0]
    super(PendulumEnv, self).__init__('./env-pendulum.html', observation_space, action_space, port, httpport, autolaunch)

class MultiplayerEnv(ThreeSimEnv):
  def __init__(self, port=9999, httpport=8765, autolaunch=True):
    observation_space = namedtuple('Box', ['high', 'low', 'shape'])
    observation_space.shape = (11,)
    observation_space.low = [-np.inf] * observation_space.shape[0]
    observation_space.high = [np.inf] * observation_space.shape[0]
    action_space = namedtuple('Box', ['high', 'low', 'shape'])
    action_space.shape = (10,)
    action_space.low = [-1.0] * action_space.shape[0]
    action_space.high = [1.0] * action_space.shape[0]
    super(MultiplayerEnv, self).__init__('./env-multiplayer.html', observation_space, action_space, port, httpport, autolaunch)
  
if __name__ == "__main__":
  env = MultiplayerEnv()
  # env.render()
  while env.running():
    env.reset()
    for i in range(200):
      action = np.zeros((10,))
      next_obs, reward, term, info = env.step(action)