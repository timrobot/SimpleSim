from aiohttp import web
from aiohttp.web_runner import GracefulExit
import aiohttp_cors
import asyncio
import websockets
import logging
import sys
import cv2
import base64
import numpy as np
import time
from multiprocessing import (
  Lock, Array, Value, RawArray, Process, Queue
)
from ctypes import c_int, c_float, c_bool, c_uint8, c_uint16, c_char
from datetime import datetime
import json
import mimetypes

# shared variables
frame_shape = (360, 640)
tx_interval = 1 / 120
envpath = ""

frame_lock = Lock()
color_buf = None
depth_buf = None
frame2_lock = Lock()
color2_buf = None
cam2_enable = Value(c_bool, False)

cmd_queue = Queue(maxsize=100)
env_queue = Queue(maxsize=100)

main_loop = None
_running = Value(c_bool, True)
last_rx_time = Array(c_char, 100)
comms_task = None

async def handle(request):
  return web.FileResponse(envpath)

mimetypes.add_type('application/x-font-ttf', '.ttf')
app = web.Application()
app.router.add_get('/', handle)
app.router.add_static('/static/', path='static', name='static')
app.router.add_static('/font/',
                       path='static/font',
                       name='font')
app.router.add_static('/js/',
                       path='static/js',
                       name='js')

cors = aiohttp_cors.setup(app, defaults={
  "*": aiohttp_cors.ResourceOptions(
    allow_credentials=True,
    expose_headers="*",
    allow_headers="*"
  )
})

async def handle_websocket(websocket, path):
  h, w = frame_shape
  color_np = np.frombuffer(color_buf, np.uint8).reshape((h, w, 3))
  depth_np = np.frombuffer(depth_buf, np.uint16).reshape((h, w))
  color2_np = np.frombuffer(color2_buf, np.uint8).reshape((h, w, 3))
  far = 50
  near = 0.1

  while _running.value:
    try:
      if not cmd_queue.empty(): # only act on step cmds
        cmd = cmd_queue.get()
        await websocket.send(json.dumps(cmd))

        data = await websocket.recv()
        if cmd["api"] == "render": continue
        res, data = data.split('$')

        last_rx_time.acquire()
        last_rx_time.value = datetime.isoformat(datetime.now()).encode()
        last_rx_time.release()

        if len(data) > 0:
          data = data.split(',')
          color = np.frombuffer(base64.b64decode(data[1]), np.uint8)
          color = cv2.imdecode(color, cv2.IMREAD_UNCHANGED)
          depth = np.frombuffer(base64.b64decode(data[3]), np.uint8)
          depth = cv2.imdecode(depth, cv2.IMREAD_UNCHANGED)
          D = depth.astype(np.float32)
          D = (D[:,:,2] / 256 + D[:,:,1]) / 256 * (far - near) + near
          D = np.where(D < 10.0, D, 0)
          depth = (D * 1000).astype(np.uint16)
          if color.shape[-1] == 4:
            color = color[:,:,:3]
          np.copyto(color_np, color)
          np.copyto(depth_np, depth)

        env_queue.put(json.loads(res))

    except websockets.ConnectionClosed:
      print("Connection closed, closing.")
      _running.value = False
      sys.exit(1)

async def request_handler(host, port):
  async with websockets.serve(handle_websocket, host, port):
    try:
      await asyncio.Future()
    except asyncio.exceptions.CancelledError:
      logging.info("Closing gracefully.")
      return
    except Exception as e:
      logging.error(e)
      sys.exit(1)

def comms_worker(path, port, httpport, run, cbuf, dbuf, flock, cbuf2, cam2_en, flock2, cmdq, envq):
  global main_loop, _running, envpath
  global color_buf, depth_buf, frame_lock
  global color2_buf, cam2_enable, frame2_lock
  global cmd_queue, env_queue
  color_buf = cbuf
  depth_buf = dbuf
  frame_lock = flock

  color2_buf = cbuf2
  cam2_enable = cam2_en
  frame2_lock = flock2

  cmd_queue = cmdq
  env_queue = envq

  envpath = path
  _running = run
  main_loop = asyncio.new_event_loop()
  request_task = main_loop.create_task(request_handler('127.0.0.1', port))
  main_task = main_loop.create_task(web._run_app(app, host="127.0.0.1", port=httpport))
  try:
    asyncio.set_event_loop(main_loop)
    main_loop.run_until_complete(main_task)
  except (KeyboardInterrupt,):
    _running.value = False
    main_loop.stop()
  finally:
    web._cancel_tasks({main_task, request_task}, main_loop)
    main_loop.run_until_complete(main_loop.shutdown_asyncgens())
    main_loop.close()

def start(path, port=9999, httpport=8765):
  global comms_task, envpath
  global color_buf, depth_buf, color2_buf

  color_buf = RawArray(c_uint8, frame_shape[0] * frame_shape[1] * 3)
  depth_buf = RawArray(c_uint8, frame_shape[0] * frame_shape[1] * 2)
  color2_buf = RawArray(c_uint8, frame_shape[0] * frame_shape[1] * 3)

  envpath = path

  comms_task = Process(target=comms_worker, args=(
    path, port, httpport, _running,
    color_buf, depth_buf, frame_lock,
    color2_buf, cam2_enable, frame2_lock,
    cmd_queue, env_queue))
  comms_task.start()

def stop():
  global comms_task
  _running.value = False
  time.sleep(0.3)
  comms_task.kill()

def read(timeout=None):
  """Return observation, reward, terminal values as well as video frames

  Returns:
      Tuple[List[float], float, bool, Dict[np.ndarray]]:
        observation, reward, terminal, { color, depth, color2 }
  """
  start_time = time.time()
  while env_queue.empty() and (timeout is None or (time.time() - start_time) < timeout):
    time.sleep(0.002)
  assert (not env_queue.empty())
  res = env_queue.get()

  h, w = frame_shape
  color_np = np.frombuffer(color_buf, np.uint8).reshape((h, w, 3))
  depth_np = np.frombuffer(depth_buf, np.uint16).reshape((h, w))
  color = np.copy(color_np)
  depth = np.copy(depth_np)
  if cam2_enable.value:
    color2_np = np.frombuffer(color2_buf, np.uint8).reshape((h, w, 3))
    color2 = np.copy(color2_np)
  else:
    color2 = None

  observation = res["obs"]
  reward = res["rew"]
  terminal = res["term"]

  return observation, reward, terminal, {
    "color": color,
    "depth": depth,
    "color2": color2
  }

def step(action):
  """Send motor values to remote location
  """
  cmd_queue.put({
    "api": "act",
    "action": [float(x) for x in action]
  })
  return read()

def reset():
  cmd_queue.put({
    "api": "reset"
  })
  return read()

def render(enable=True):
  cmd_queue.put({
    "api": "render",
    "value": enable
  })

def running():
  return _running.value

if __name__ == "__main__":
  start()
  reset()
  while running():
    obs, rew, term, _ = step([0] * 10)
    time.sleep(0.1)