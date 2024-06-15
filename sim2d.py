import numpy as np
from scipy.spatial.transform import Rotation
from collections import namedtuple
import cv2

def in2m(x):
  return x * 0.0254

class TennisBallClawbot2dEnv:
  def __init__(self):
    self.chassis = namedtuple('Mesh', ['position', 'rotation'])
    self.chassis.position = np.zeros([3], np.float32)
    self.chassis.position[2] = in2m(2.75)
    self.chassis.rotation = 0

    self.ball = namedtuple('Mesh', ['position', 'rotation'])
    self.ball.position = np.zeros([3], np.float32)
    self.ball.position[2] = in2m(1.28)

    self.observation_space = namedtuple('Box', ['high', 'low', 'shape'])
    self.observation_space.shape = (10,)
    self.observation_space.low = [-np.inf] * self.observation_space.shape[0]
    self.observation_space.high = [np.inf] * self.observation_space.shape[0]
    self.action_space = namedtuple('Box', ['high', 'low', 'shape'])
    self.action_space.shape = (10,)
    self.action_space.low = [-1.0] * self.action_space.shape[0]
    self.action_space.high = [1.0] * self.action_space.shape[0]

  def generate_observation(self):
    return [
      self.chassis.position[0],
      self.chassis.position[1],
      self.chassis.rotation, # ccw starting at North
      -np.pi / 4,
      0.0,
      self.ball.position[0],
      self.ball.position[1],
      self.clawBase.position[0] - self.ball.position[0],
      self.clawBase.position[1] - self.ball.position[1],
      self.clawBase.position[2] - self.ball.position[2]
    ]

  @property
  def clawBase(self):
    z = self.chassis.rotation
    cz = np.cos(z)
    sz = np.sin(z)
    mesh = namedtuple('Mesh', ['position', 'rotation'])
    mesh.position = in2m(5) * np.array([-sz, cz, 0]) + self.chassis.position
    mesh.rotation = z
    return mesh

  def reset(self):
    self.chassis.position = np.clip(np.random.randn(3), -1, 1)
    self.chassis.position[2] = in2m(2.75)
    self.chassis.rotation = np.random.random() * np.pi * 2 - np.pi
    self.ball.position = np.random.random([3]) * in2m(144) - in2m(72)
    self.ball.position[2] = in2m(1.28)
    return self.generate_observation()

  def step(self, action, timestep=0.1):
    action = np.clip(action, self.action_space.low, self.action_space.high)
    rpm = 100
    omega = rpm * 2 * np.pi / 60
    meters_psec = omega * in2m(2)
    linearCoeff = meters_psec
    angularCoeff = meters_psec / in2m(10)
    # linearCoeff = 0.25 # goes a bit slower than turning
    # angularCoeff = 1.0

    leftAction = action[0]
    rightAction = -action[9]
    angular_vel = angularCoeff * (rightAction - leftAction)
    linear_vel = linearCoeff * (leftAction + rightAction) / 2

    yaw = self.chassis.rotation + angular_vel * timestep
    while yaw > np.pi: yaw -= 2 * np.pi
    while yaw < -np.pi: yaw += 2 * np.pi
    self.chassis.rotation = yaw

    cz = np.cos(yaw)
    sz = np.sin(yaw)
    forward = np.array([-sz, cz, 0], np.float32)
    self.chassis.position += forward * linear_vel * timestep
    self.chassis.position = np.clip(self.chassis.position, -2.5, 2.5)

    pq = self.ball.position - self.clawBase.position
    distance = np.linalg.norm(pq)
    pq_theta = np.arctan2(-pq[0], pq[1])
    dtheta = pq_theta - yaw
    while dtheta > np.pi: dtheta -= 2 * np.pi
    while dtheta < -np.pi: dtheta += 2 * np.pi
    reward = 1 - distance - np.abs(dtheta) / np.pi
    obs = self.generate_observation()
    return obs, reward, False, {}
  
  def render(self):
    canvas = np.ones((500, 500, 3), np.uint8) * 255
    xyz2coord = lambda xyz: (int(xyz[0] * 100 + 250), int(250 - xyz[1] * 100))
    cv2.circle(canvas, xyz2coord(self.chassis.position), int(in2m(5) * 100), [0, 0, 255], -1)
    cv2.circle(canvas, xyz2coord(self.clawBase.position), int(in2m(2) * 100), [0, 0, 255], -1)
    cv2.circle(canvas, xyz2coord(self.ball.position), int(in2m(1.28) * 100), [0, 255, 0], -1)
    cv2.circle(canvas, xyz2coord(self.ball.position), int(in2m(5) * 100), [255, 0, 0], 1)
    cv2.imshow("sim2d", canvas)
    cv2.waitKey(1)

  def running(self):
    return True

if __name__ == "__main__":
  env = TennisBallClawbot2dEnv()
  while env.running():
    env.reset()
    for i in range(200):
      action = [1, 0, 0, 0, 0, 0, 0, 0, 0, -.5]
      next_obs, reward, term, _ = env.step(action)
      print(next_obs[:2], np.degrees(next_obs[2]))
      env.render()