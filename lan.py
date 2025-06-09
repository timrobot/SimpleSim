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
import psutil
import subprocess
import platform
import os
import signal

# shared variables
frame_shape = (360, 640)
tx_interval = 1 / 120
envpath = ""

frame_lock = Lock()
color_buf = None
depth_buf = None

cmd_queue = Queue(maxsize=100)
env_queue = Queue(maxsize=100)

main_loop = None
_running = Value(c_bool, False)
last_rx_time = Array(c_char, 100)
comms_task = None

async def handle(request):
  return web.FileResponse(envpath)

app = web.Application()
app.router.add_get('/', handle)
app.router.add_static('/static/', path='static', name='static')
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
  far = 50
  near = 0.1

  while _running.value:
    try:
      if not cmd_queue.empty(): # only act on step cmds
        cmd = cmd_queue.get()
        await websocket.send(json.dumps(cmd))
        # we want to process the cmd through mujoco's physx engine

        data = await websocket.recv()

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
          if color.shape[-1] == 4: # sRGB
            color = color[:,:,:3]
          np.copyto(color_np, color)
          np.copyto(depth_np, depth)

        env_queue.put({})

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

def recursive_transform_float(obj):
  if isinstance(obj, dict):
    for k, v in obj.items():
      obj[k] = recursive_transform_float(v)
  elif isinstance(obj, np.ndarray):
    obj = obj.tolist()
    for i, v in enumerate(obj):
      obj[i] = recursive_transform_float(v)
  elif isinstance(obj, list):
    for i, v in enumerate(obj):
      obj[i] = recursive_transform_float(v)
  elif isinstance(obj, float) or isinstance(obj, int):
    return float(int(obj * 10000)) / 10000
  return obj

def comms_worker(path, port, httpport, run, cbuf, dbuf, flock, cmdq, envq):
  global main_loop, _running, envpath
  global color_buf, depth_buf, frame_lock
  global cmd_queue, env_queue
  color_buf = cbuf
  depth_buf = dbuf
  frame_lock = flock

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

def kill_all_processes_by_port(port):
  killed_any = False

  if platform.system() == 'Windows':
    def kill_process(proc):
        print(f"Killing process with PID {proc.pid}")
        proc.kill()

    def kill_process_and_children(proc):
      children = proc.children(recursive=True)
      for child in children:
          kill_process(child)

      kill_process(proc)

    for proc in psutil.process_iter(['pid', 'name']):
      try:
          connections = proc.net_connections()
          for conn in connections:
              print(conn)
              if conn.laddr.port == port:
                  print(f"Found process with PID {proc.pid} and name {proc.info['name']}")
                  kill_process_and_children(proc)
      except (psutil.AccessDenied, psutil.NoSuchProcess):
          print(f"Access denied or process does not exist: {proc.pid}")

  elif platform.system() == 'Darwin' or platform.system() == 'Linux':
    command = f"netstat -tlnp | grep {port}"
    c = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr = subprocess.PIPE)
    stdout, stderr = c.communicate()
    proc = stdout.decode().strip().split(' ')[-1]
    try:
      pid = int(proc.split('/')[0])
      os.kill(pid, signal.SIGKILL)
      killed_any = True
    except Exception as e:
      pass

  return killed_any

def start(path, port=9999, httpport=8765):
  global comms_task, envpath
  global color_buf, depth_buf

  kill_all_processes_by_port(httpport)
  kill_all_processes_by_port(port)

  color_buf = RawArray(c_uint8, frame_shape[0] * frame_shape[1] * 3)
  depth_buf = RawArray(c_uint8, frame_shape[0] * frame_shape[1] * 2)

  envpath = path

  _running.value = True
  comms_task = Process(target=comms_worker, args=(
    path, port, httpport, _running,
    color_buf, depth_buf, frame_lock,
    cmd_queue, env_queue))
  comms_task.start()

def stop():
  global comms_task
  _running.value = False
  if comms_task is not None:
    comms_task.kill()

def render(body_info, timeout=None):
  """Return video frames

  Returns:
      Tuple[np.ndarray]: color, depth
  """
  if not _running.value: start()

  cmd_queue.put({
    "api": "render",
    "bodies": recursive_transform_float(body_info)
  })

  start_time = time.time()
  while env_queue.empty() and (timeout is None or (time.time() - start_time) < timeout):
    time.sleep(0.001)
  assert (not env_queue.empty())
  res = env_queue.get()

  h, w = frame_shape
  color_np = np.frombuffer(color_buf, np.uint8).reshape((h, w, 3))
  depth_np = np.frombuffer(depth_buf, np.uint16).reshape((h, w))
  color = np.copy(color_np)
  depth = np.copy(depth_np)

  return color, depth

def running():
  return _running.value

if __name__ == "__main__":
  import pyrr
  if not running(): start('./env/pendulum.html')
  while running():
    color, depth = render([{
      'name': 'pendulum_joint',
      'position': [0, 0.5, 0], # [x, z, -y]
      'quaternion': pyrr.Quaternion.from_z_rotation(np.radians(time.time() * 180)) # we are still using three js axes for this
    }])
    cv2.imshow('color', color)
    cv2.waitKey(1)
    # time.sleep(0.1)