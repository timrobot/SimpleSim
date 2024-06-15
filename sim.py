
import numpy as np
import customtkinter as ctk
from multiprocessing import (
  Process,
  Lock,
  Array,
  RawArray,
  Value
)
from ctypes import c_uint8
from PIL import Image, ImageTk
import logging
import lan
import time
import webbrowser
from collections import namedtuple
import sys
import cv2

def _depth2rgb(depth):
  return cv2.applyColorMap(np.sqrt(depth).astype(np.uint8), cv2.COLORMAP_HSV)

def _ctk_interface(key_values,color_buf,depth_buf,color2_buf):
  ctk.set_appearance_mode("Dark")
  ctk.set_default_color_theme("blue")
  app = ctk.CTk()
  app.geometry("1600x900")

  h, w = lan.frame_shape
  print(color_buf)
  print(np.uint8)
  color_np = np.frombuffer(color_buf, np.uint8).reshape((h, w, 3))
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
    app.after(4, animate)

    depth_image = Image.fromarray(_depth2rgb(depth_np))
    color_image = Image.frombuffer('RGB', (w, h), color_buf, 'raw')
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

  def on_key_press(event):
    charcode = ord(event.char) if event.char else None
    if charcode and charcode > 0 and charcode < 128:
      keycodes[event.keycode] = charcode
      key_values[charcode] = 1

  def on_key_release(event):
    charcode = None
    if event.keycode in keycodes: charcode = keycodes[event.keycode]
    if charcode and charcode > 0 and charcode < 128:
      key_values[charcode] = 0

  app.bind("<KeyPress>", on_key_press)
  app.bind("<KeyRelease>", on_key_release)
  app.after(16, animate)
  app.mainloop()
  lan.stop()

class ThreeSimEnv:
  def __init__(self, observation_space=None, action_space=None, port=9999, httpport=8765):
    """Remote Interface showing the data coming in from the robot

    Args:
        host (str, optional): host ip of the robot. Defaults to "0.0.0.0".
    """
    lan.start(port, httpport, observation_space.shape[0], action_space.shape[0])
    self.keyboard_buf = RawArray(c_uint8, 128)
    time.sleep(0.3)
    self.browser_process = webbrowser.open(f"http://127.0.0.1:{httpport}")
    self.ui_task = None

    # OpenAI Gym convenience fields
    self._steps = 0
    self.max_steps = 1000
    self.observation_space = observation_space
    self.action_space = action_space

  def __del__(self):
    lan.stop()
    if self.ui_task:
      self.ui_task.kill()
    sys.exit(1)

  @property
  def keys(self):
    return {
      chr(x): self.keyboard_buf[x] for x in range(128)
    }

  def read(self):
    """Read sensor values from the robot, including color and depth

    Returns:
        (np.ndarray, np.ndarray, np.ndarray, dict): color, depth, other sensor values
    """
    return lan.read()
  
  def running(self):
    return lan.running()
  
  def step(self, values):
    """
    Convenience function to act like OpenAI Gym step(), since setting motor values does
    not actually write motor values due to the Queue command system in simulation
    """
    assert(len(values) == self.action_space.shape[0])
    if not lan.running():
      raise Exception("Environment has been torn down.")
    self._steps += 1

    lan.write(values)
    time.sleep(0.05) # wait a tiny bit to get some observation propagation
    color, depth, sensors, info = lan.read()
    reward = info["reward"]
    terminal = self._steps == self.max_steps

    return sensors, reward, terminal, {
      "color": color,
      "depth": depth,
      "color2": info["cam2"],
      "time": info["time"]
    }

  def reset(self, full_obs=False):
    """
    Convenience function to act like OpenAI Gym reset()
    """
    if not lan.running():
      raise Exception("Environment has been torn down.")
    
    lan.reset()
    self._steps = 0
    time.sleep(0.5) # wait a bit for the simulation to reset
    # usually it better to run the last action that you were going to do,
    # then process the current observation and label it as the "next" observation

    color, depth, sensors, info = lan.read()
    while len(sensors) == 0:
      color, depth, sensors, info = lan.read()
    info.pop("reward")
    return sensors if not full_obs else {
      "sensors": sensors,
      "color": color,
      "depth": depth,
      "color2": info["cam2"],
      "time": info["time"]
    }
  
  def render(self):
    if not self.ui_task:
      while lan.color_buf is None:
        continue
      print("Value: ", lan.color_buf)
      self.ui_task = Process(target=_ctk_interface, args=(self.keyboard_buf,lan.color_buf,lan.depth_buf,lan.color2_buf))
      self.ui_task.start()

  # convenience functions
  @property
  def motor(self):
    return lan.motor_values
  
  def read(self):
    return lan.read()
  
class TennisBallClawbotEnv(ThreeSimEnv):
  def __init__(self, port=9999, httpport=8765):
    observation_space = namedtuple('Box', ['high', 'low', 'shape'])
    observation_space.shape = (10,)
    observation_space.low = [-np.inf] * observation_space.shape[0]
    observation_space.high = [np.inf] * observation_space.shape[0]
    action_space = namedtuple('Box', ['high', 'low', 'shape'])
    action_space.shape = (10,)
    action_space.low = [-1.0] * action_space.shape[0]
    action_space.high = [1.0] * action_space.shape[0]
    lan.filename = './env-tennis-ball-clawbot.html'
    super(TennisBallClawbotEnv, self).__init__(observation_space, action_space, port, httpport)
  
class PendulumEnv(ThreeSimEnv):
  def __init__(self, port=9999, httpport=8765):
    observation_space = namedtuple('Box', ['high', 'low', 'shape'])
    observation_space.shape = (3,)
    observation_space.low = [-np.inf] * observation_space.shape[0]
    observation_space.high = [np.inf] * observation_space.shape[0]
    action_space = namedtuple('Box', ['high', 'low', 'shape'])
    action_space.shape = (1,)
    action_space.low = [-1.0] * action_space.shape[0]
    action_space.high = [1.0] * action_space.shape[0]
    lan.filename = './env-pendulum.html'
    super(PendulumEnv, self).__init__(observation_space, action_space, port, httpport)

if __name__ == "__main__":
  env = TennisBallClawbotEnv()
  while env.running():
    obs = env.reset()
    for i in range(200):
      y = env.keys["w"] - env.keys["s"]
      x = env.keys["d"] - env.keys["a"]
      action = [0] * 10
      action[0] = y + x
      action[9] = -y + x
      action[1] = env.keys["p"] - env.keys["l"]
      next_obs, reward, term, _ = env.step(action)
      env.render()
      obs = next_obs