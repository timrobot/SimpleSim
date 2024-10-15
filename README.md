# SimpleSim
This environment is a simple simulation engine to view and control a standard VEX Clawbot and move it via motor torques, similar to the interface used on the Jetson Orin Nano. This is in __BETA__ and must be run from the repository directory until a package can be created and some features added, however feel free to test as you like.

__Sim details:__ three.js render, ammo/bullet physics

__OSHW Support:__ Windows/Mac/Linux, does not require a GPU

### Quickstart
To install the needed libraries, you will first need tkinter. Note that while it comes built in on Windows, Mac/Linux will require an additional install.

```bash
# Linux
sudo apt-get install python3-tk
```

```bash
# Mac
brew install python-tk
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

### Example code
A full example code to manually control the robot can be seen here:

```python
from cortano import RealsenseCamera, VexV5

if __name__ == "__main__":
  camera = RealsenseCamera()
  robot = VexV5()

  while robot.running():
    color, depth = camera.read()
    sensors, battery = robot.read()

    keys = robot.controller.keys
    y = keys["w"] - keys["s"]
    x = keys["d"] - keys["a"]
    robot.motors[0] = (y + x) * 50
    robot.motors[9] = (y - x) * 50
    robot.motors[2] = (keys["p"] - keys["l"]) * 50
    robot.motors[7] = (keys["o"] - keys["k"]) * 50
```

### Description

A cube is positioned randomly inside of a standard 144"x144" field, with a controllable VEX clawbot.
|  |  |
| -- | -- |
| Action Space | Box(-1.0, 1.0, (10,), float32) |
| Observation Shape | (12,) |
| Observation High | [inf inf inf inf inf inf inf inf inf inf inf inf] |
| Observation Low | [-inf -inf -inf -inf -inf -inf -inf -inf -inf -inf -inf -inf] |
| Import | `from cortano import VexV5, RealsenseCamera` |

### Action Space
Some actions are left intentionally blank to reflect the VEX microcontroller's disconnected ports.

| Num | Action | Control Min | Control Max | Unit |
| --- | ------ | ----------- | ----------- | ---- |
| 0 | Angular velocity target of the left motor | -100 | 100 | percent |
| 1 | ❌ |  |  |  |
| 2 | Angular velocity target of the claw | -100 | 100 | percent |
| 3 | ❌ |  |  |  |
| 4 | ❌ |  |  |  |
| 5 | ❌ |  |  |  |
| 6 | ❌ |  |  |  |
| 7 | Angular velocity target of the arm | -100 | 100 | percent |
| 8 | ❌ |  |  |  |
| 9 | Angular velocity target of the right motor | -100 | 100 | percent |

### Observation Space

#### VexV5 Clawbot
`sensors, battery = robot.read()`
| Num | Action | Min | Max | Unit |
| --- | ------ | --- | --- | ---- |
| 0  | Left Motor ang position | -inf | inf | position (degrees) |
| 1  | Left Motor ang velocity | -inf | inf | velocity (degrees/second) |
| 2  | Left Motor torque | -inf | inf | Nm * 1e3 |
| 3  | Right Motor ang position | -inf | inf | position (degrees) |
| 4  | Right Motor ang velocity | -inf | inf | velocity (degrees/se5ond) |
| 5  | Right Motor torque | -inf | inf | Nm * 1e3 |
| 6  | Arm ang position | -inf | inf | position (degrees) |
| 7  | Arm ang velocity | -inf | inf | velocity (degrees/second) |
| 8  | Arm torque | -inf | inf | Nm * 1e3 |
| 9  | Claw ang position | -inf | inf | position (degrees) |
| 10 | Claw ang velocity | -inf | inf | velocity (degrees/second) |
| 11 | Claw torque | -inf | inf | Nm * 1e3 |

#### RealsenseCamera
`color, depth = camera.read()`
| Name | Description | Shape | MaxValue | dtype |
| --- | ------ | --- | --- | ---- |
| color | BGR numpy array | (360, 640, 3) | 255 | uint8 |
| depth | Depth(m) numpy array | (360, 640) | 10000 | uint16 |

#### Camera API
`camera = RealsenseCamera()`
| Property | Description |
| --- | --- |
| `camera.fx` | focal length (x axis) |
| `camera.fy` | focal length (y axis) |
| `camera.cx` | principal center point (x axis) |
| `camera.cy` | principal center point (y axis) |

### Environmental Details
The default unit is in meters, with the Z axis pointing upwards.

Color and depth frames are gathered from the Realsense Camera, which tries to mimic a Realsense D415. Note that by default, the simulator will display these frames on the control window. To manually control the robot, you can call the `robot.controller.keys[keyname:str]` api. For example

```python
robot = VexV5()
while robot.running():
  a_pressed = robot.controller.keys['a'] # 0 or 1
```

Note that rendering the color and depth frames take a significant amount of time from the simulator, so it will cause step times to increase as well.
To turn this off, set render to False:

```python
robot = VexV5(render=False)
```

#### Aprilcube Details

Each of the cube's 6 faces contains one of three apriltags - tag16h5 {12, 13, 14}. The cube is 2.56"x2.56"x2.56", however when detecting tags remember that apriltags detect only the inner (black) portion. That means that in reality the tag's size parameter should be 2.56 * 3/4.
