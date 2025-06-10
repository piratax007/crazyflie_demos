# Crazyflie Demos
A set of ROS2 humble nodes and scripts to test Crazyflie using different control strategies. All these nodes are tested using a Crazyflie 2.x drone by Bitcraze.

This is a work in progress.

## Installation
### Dependencies
The nodes and scripts in this package utilize Crazyswarm 2 [1] to comunicate the VICON motion capture system with the Crazyflie drone.

Clone this repository into your `ros2_ws/src` directory and install the package:
```
colcon build --packages-select crazyflie_demos
```

## How to
> Remember source your ros installation in any new terminal.

### Simple Crazyflie test -> takeoff and landing
The `crazyflie_teleop` node utilize the `teleop` package to control the takeoff and landing of the Crazlyflie.

**Terminal 1:**
```
ros2 launch crazyflie launch.py
```

**Terminal 2:**
```
ros2 run crazyflie_demos crazyflie_teleop
```

**Terminal 3:**
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- Press t: take off and hover at 1m
- Press b: land

### Testing the NN controller and loggind data using the uSD deck
```
ros2 run crazyflie_demos crazyflie_NN_controller -n True -l True -d 1
```

- Parameter `-n True/False` activate/deactivate the nn computation and the motor control through the nn actions.
- Parameter `-l True/False` activate/deactivate the log in the usd card onboard.
- Parameter `-d time` set the delay between the activation of the nn and the activation of the loggin. This parameter mustn't be zero. 


[1]: https://imrclab.github.io/crazyswarm2/