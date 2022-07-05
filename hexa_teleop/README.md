# hexa_teleop
Package to teleoperate a DJI550 Hexacopter drone. 
By default uses a Logitech F710. To use with Sony PS4:

`export JOYSTICK_MODEL=ps4`

Connect the gamepad to teleoperation laptop and configure it as a client of ros master running on the drone adding to .bashrc:
```
export ROS_MASTER_URI=http://<remotehost_ip>:11311
export ROS_IP=<localhost_ip>
```
It will publish setpoint velocities and mode=0 when in teleoperation mode or mode=1 in autonomous mode. Check launch file for topic names.

## Default setup
| Gamepad | Description |
| --- | --- |
|L1(ps4) or LB(F710)  | Deadman buttom for teleoperation|
| Right stick| |
|  | UP = move forward|
|  | DOWN = move backward|
|  |  LEFT = move left|
|  | RIGHT = move right|
|Left stick| |
|  | UP = increase altitude|
|  | DOWN = decrease altitude|
|  | LEFT = yaw rotation to the left|
|  | RIGHT = yaw rotation to the right|
|R1(ps4) or RB(F710)  | Deadman buttom for autonomous mode|