# SimpleSim
This environment is a simple simulation engine to view and control a standard VEX Clawbot and move it via motor torques. This is in <b>BETA</b> and must be run from the repository directory until a package can be created and some features added, however feel free to test as you like.

#### Quickstart
To install the needed libraries, you will first need tkinter. Note that while it comes built in on many systems, some systems will require an additional install. For instance, on ubuntu you can custom install tkinter by typing in the following command:

```bash
sudo apt-get install python3-tk
```

For all platforms, you can then do a standard pip install.

```bash
git clone https://github.com/timrobot/SimpleSim.git
cd SimpleSim
python3 -m pip install -r requirements.txt
```

Run an example program to control the robot. You can use W(↑) A(←) S(↓) D(→) to control the robot navigation and P(arm ↑) L(arm ↓) and O(claw →←) K(claw ←→) to control the arm and claws.

```bash
python3 main.py
```

### Example
You will notice that the interface to the simulation engine is very similar to Gym's interface, for good reason. It's quite simple to read, and easy to get up and running. For example, to import the tennis ball clawbot environment:

```python
from sim3d import TennisBallClawbotEnv

if __name__ == "__main__":
  env = TennisBallClawbotEnv()
  env.render()
  while env.running():
    obs = env.reset()
    for step in range(200):
      action = [0] * 10
      action[0] = 0.5 # left
      action[9] = -0.5 # right

      new_obs, reward, term, info = env.step(action)
```

### Description

#### Tennis Ball Clawbot
A ball is positioned randomly inside of a standard 144"x144" field. The goal of the robot is to navigate to the position of the ball such that the ball is at the center of the claw. This environment is made with Three.js using Ammo/Bullet physics. It runs on Windows/Mac/Linux and does not require a GPU.

|  |  |
| -- | -- |
| Action Space | Box(-1.0, 1.0, (10,), float32) |
| Observation Shape | (10,) |
| Observation High | [inf inf inf inf inf inf inf inf inf inf] |
| Observation Low | [-inf -inf -inf -inf -inf -inf -inf -inf -inf -inf] |
| Import | `from sim3d import TennisBallClawbotEnv` |

#### Aprilcube Clawbot
|  |  |
| -- | -- |
| Action Space | Box(-1.0, 1.0, (10,), float32) |
| Observation Shape | (6,) |
| Observation High | [inf inf inf inf inf inf] |
| Observation Low | [-inf -inf -inf -inf -inf -inf] |
| Import | `from sim3d import AprilcubeClawbotEnv` |

### Action Space
Some actions are left intentionally blank to reflect the VEX microcontroller's disconnected ports.

| Num | Action | Control Min | Control Max | Unit |
| --- | ------ | ----------- | ----------- | ---- |
| 0 | Angular velocity target of the left motor | -1 | 1 | rpm (rad/m) |
| 1 | Torque applied on the arm | -1 | 1 | Torque (Nm) |
| 2 | Torque applied on the claw | -1 | 1 | Torque (Nm) |
| 3 | ❌ |  |  |  |
| 4 | ❌ |  |  |  |
| 5 | ❌ |  |  |  |
| 6 | ❌ |  |  |  |
| 7 | ❌ |  |  |  |
| 8 | ❌ |  |  |  |
| 9 | Angular velocity target of the right motor | -1 | 1 | rpm (rad/m) |

### Observation Space

#### Tennis Ball Clawbot
| Num | Action | Min | Max | Unit |
| --- | ------ | --- | --- | ---- |
| 0 | Chassis position x | -1.83 | 1.83 | position (m) |
| 1 | Chassis position y | -1.83 | 1.83 | position (m) |
| 2 | Counter-clockwise heading of chassis, y-axis=0 | -𝜋 | 𝜋 | radians |
| 3 | Pitch of the arm | -𝜋/2 | 𝜋/2 | radians |
| 4 | Arm angular velocity | -inf | inf | rad/s |
| 5 | Ball position x | -1.83 | 1.83 | position (m) |
| 6 | Ball position y | -1.83 | 1.83 | position (m) |
| 7 | Ball distance from claw | 0 | 5.18 | distance (m) |
| 8 | Counter-clockwise heading relative to ball | -𝜋 | 𝜋 | radians |
| 9 | Ball detected in claw | 0 | 1 | on/off |

#### Aprilcube Clawbot
| Num | Action | Min | Max | Unit |
| --- | ------ | --- | --- | ---- |
| 0 | Chassis position x | -1.83 | 1.83 | position (m) |
| 1 | Chassis position y | -1.83 | 1.83 | position (m) |
| 2 | Counter-clockwise heading of chassis, y-axis=0 | -𝜋 | 𝜋 | radians |
| 3 | Pitch of the arm | -𝜋/2 | 𝜋/2 | radians |
| 4 | Arm angular velocity | -inf | inf | rad/s |
| 5 | Cube detected in claw | 0 | 1 | on/off |

### Reward
A reward is given based on the distance the claw is from the object, as well as the difference of the claw's orientation from the direction from the claw to the object.

`reward = 1 - sum(object.xyz - claw.xyz).sqrt() - abs(heading - claw.angleto(object)) / PI`

This is the default reward, however you may specify your own reward function as you like.

### Environmental Details
Color and depth frames are shown on a window when `render()` gets called. This window also allows keyboard inputs to be queried, meaning that if you wanted to
manually control the robot, you can call the `env.keys[keyname:str]` api. For example

```python
env.render()
while env.running():
  a_pressed = env.keys['a'] # 0 or 1
```

To access these color and depth frames, say, for computer vision and SLAM, you can use the 4th output from the `step()` function, as well as enable the info flag in `reset(extra_info=True)`:

```python
import cv2
from sim3d import TennisBallClawbotEnv

if __name__ == "__main__":
  env = TennisBallClawbotEnv()
  env.render()
  obs, info = env.reset(extra_info=True)
  while env.running():
    obs, reward, term, info = env.step([0] * 10)
    color = info["color"]
    depth = info["depth"]

    cv2.imshow("color", color)
    cv2.waitKey(1)
```

Note that rendering the color and depth frames take a significant amount of time from the simulator, so it will cause step times to increase as well.

#### Aprilcube Clawbot

Each of the cube's 6 faces contains one of three apriltags - tag16h5 {12, 13, 14}. The cube is 2.56"x2.56"x2.56", however when detecting tags remember that apriltags detect only the inner (black) portion. That means that in reality the tag's size parameter should be 2.56 * 3/4.
