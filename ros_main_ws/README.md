# version 0.1.0
* We assume that roscore is running
* we assume that `source devel/setup.bash` has been run

# Motor Control

Bring up the motor controller (Python)

```bash
rosrun motors motor_node.py
```

View active topic by:

```bash
rostopic list
```

# Using the joystick to control the robot:

```
sudo ./sixpair
sudo sixad --start
Starting bluetooth (via systemctl): bluetooth.service.
sixad-bin[23148]: started
sixad-bin[23148]: sixad started, press the PS button now
sixad-sixaxis[23152]: started
sixad-sixaxis[23152]: Connected 'PLAYSTATION(R)3 Controller (64:D4:BD:0F:F9:75)' [Battery 05]
```

Then, run the joystic node:

```bash
rosrun joystick_node joystick_node
```


