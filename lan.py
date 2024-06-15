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
# from scipy.signal import convolve2d

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

cmd_values = Queue(maxsize=100)
motor_values = None
sensor_values = None
sensor_length = Value(c_int, 0)
reward_value = Value(c_float, 0)

main_loop = None
_running = Value(c_bool, True)
last_rx_time = Array(c_char, 100)
comms_task = None

async def handle(request):
  return web.FileResponse(envpath)

app = web.Application()
app.router.add_get('/', handle)
app.router.add_static('/static/', path='./static', name='static')

cors = aiohttp_cors.setup(app, defaults={
  "*": aiohttp_cors.ResourceOptions(
    allow_credentials=True,
    expose_headers="*",
    allow_headers="*"
  )
})

async def receiver(websocket):
  h, w = frame_shape
  color_np = np.frombuffer(color_buf, np.uint8).reshape((h, w, 3))
  depth_np = np.frombuffer(depth_buf, np.uint16).reshape((h, w))
  color2_np = np.frombuffer(color2_buf, np.uint8).reshape((h, w, 3))
  far = 50
  near = 0.1

  while _running.value:
    try:
      data = await websocket.recv()
      obsrew, data = data.split('$')

      last_rx_time.acquire()
      last_rx_time.value = datetime.isoformat(datetime.now()).encode()
      last_rx_time.release()

      obsrew = json.loads(obsrew)
      sensor_values.acquire()
      ns = sensor_length.value = len(obsrew['obs'])
      sensor_values[:ns] = obsrew['obs']
      sensor_values.release()
      reward_value.acquire()
      reward_value.value = obsrew['rew']
      reward_value.release()

      data = data.split(',')
      color = np.frombuffer(base64.b64decode(data[1]), np.uint8)
      color = cv2.imdecode(color, cv2.IMREAD_UNCHANGED)
      color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
      depth = np.frombuffer(base64.b64decode(data[3]), np.uint8)
      depth = cv2.imdecode(depth, cv2.IMREAD_UNCHANGED)
      D = depth.astype(np.float32)
      D = (D[:,:,2] / 256 + D[:,:,1]) / 256 * (far - near) + near
      D = np.where(D < 10.0, D, 0)
      depth = (D * 1000).astype(np.uint16)
      np.copyto(color_np, color)
      np.copyto(depth_np, depth)

    except websockets.ConnectionClosed:
      print("Connection closed, closing.")
      _running.value = False
      sys.exit(1)

async def sender(websocket):
  last_tx_time = None
  while _running.value:
    try:
      if not cmd_values.empty():
        cmd = cmd_values.get()
        await websocket.send(json.dumps(cmd))
      else:
        # await asyncio.sleep(tx_interval)
        # continue
        curr_time = datetime.now()
        dt = tx_interval if last_tx_time is None else (curr_time - last_tx_time).total_seconds()
        if dt < tx_interval:
          await asyncio.sleep(tx_interval - dt)
          last_tx_time = datetime.now()
        else:
          last_tx_time = curr_time
        motor_values.acquire()
        motors = motor_values[:]
        motor_values.release()
        # over a simulation, we want the output to be a float: [-1, 1]
        motors = [float(int(x * 10000)) / 10000 for x in np.array(motors, np.float32).clip(-1, 1)]
        await websocket.send(json.dumps({
          "api": "act",
          "action": motors
        }))
    except websockets.ConnectionClosed:
      print("Connection closed, closing.")
      _running.value = False
      sys.exit(1)

async def handle_websocket(websocket, path):
  recv_task = asyncio.create_task(receiver(websocket))
  send_task = asyncio.create_task(sender(websocket))
  await asyncio.gather(recv_task, send_task)

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

def comms_worker(path, port, httpport, run, cbuf, dbuf, flock, cbuf2, cam2_en, flock2, mvals, svals, ns, rew, cmd):
  global main_loop, _running, envpath
  global color_buf, depth_buf, frame_lock
  global color2_buf, cam2_enable, frame2_lock
  global motor_values, sensor_values, sensor_length, reward_value, cmd_values
  color_buf = cbuf
  depth_buf = dbuf
  frame_lock = flock

  color2_buf = cbuf2
  cam2_enable = cam2_en
  frame2_lock = flock2

  cmd_values = cmd
  motor_values = mvals
  sensor_values = svals
  sensor_length = ns
  reward_value = rew

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

def start(path, port=9999, httpport=8765, num_sensors=20, num_motors=10):
  global comms_task, envpath
  global color_buf, depth_buf, color2_buf
  global motor_values, sensor_values, sensor_length

  color_buf = RawArray(c_uint8, frame_shape[0] * frame_shape[1] * 3)
  depth_buf = RawArray(c_uint8, frame_shape[0] * frame_shape[1] * 2)
  color2_buf = RawArray(c_uint8, frame_shape[0] * frame_shape[1] * 3)

  motor_values = Array(c_float, num_motors)
  sensor_values = Array(c_float, num_sensors)
  # sensor_length.value = num_sensors
  envpath = path

  comms_task = Process(target=comms_worker, args=(
    path, port, httpport, _running,
    color_buf, depth_buf, frame_lock,
    color2_buf, cam2_enable, frame2_lock,
    motor_values, sensor_values, sensor_length,
    reward_value, cmd_values))
  comms_task.start()

def stop():
  global comms_task
  _running.value = False
  time.sleep(0.3)
  comms_task.kill()

def read():
  """Return sensors values, voltage (V) and time when the data comes in

  Returns:
      Tuple[np.ndarray, np.ndarray, List[int], Dict[float, datetime, np.ndarray]]: color, depth, sensors, { voltage, time, cam2 }
  """
  h, w = frame_shape
  color_np = np.frombuffer(color_buf, np.uint8).reshape((h, w, 3))
  depth_np = np.frombuffer(depth_buf, np.uint16).reshape((h, w))
  # frame_lock.acquire()
  color = np.copy(color_np)
  depth = np.copy(depth_np)
  # frame_lock.release()
  # frame2_lock.acquire()
  if cam2_enable.value:
    color2_np = np.frombuffer(color2_buf, np.uint8).reshape((h, w, 3))
    color2 = np.copy(color2_np)
  else:
    color2 = None
  # frame2_lock.release()

  sensor_values.acquire()
  ns = sensor_length.value
  sensors = [] if ns == 0 else sensor_values[:ns]
  reward = reward_value.value
  sensor_values.release()

  last_rx_time.acquire()
  rxtime = last_rx_time.value.decode()
  if len(rxtime) > 0:
    rxtime = datetime.fromisoformat(rxtime)
  else:
    rxtime = None
  last_rx_time.release()

  return color, depth, sensors, { "reward": reward, "time": rxtime, "cam2": color2 }

def write(values):
  """Send motor values to remote location
  """
  values = [float(x) for x in values]
  motor_values.acquire()
  motor_values[:min(len(motor_values), len(values))] = values[:min(len(motor_values), len(values))]
  motor_values.release()
  # cmd_values.put({
  #   "api": "act",
  #   "action": values
  # })

def reset():
  cmd_values.put({
    "api": "reset"
  })

def running():
  return _running.value

if __name__ == "__main__":
  start()
  while running():
    color, depth, sensors, _ = read()
    time.sleep(0.1)