# A Global-Local Planning Framework for Continuous Frontier-Based Robot Exploration

## Installation

Place this package inside the `src` folder of your ROS 2 workspace.

Build the workspace.

```bash
colcon build
```

## Source the workspace.


## Run in Simulation

## Open a terminal and source the workspace.

Launch the simulation system.

```bash
ros2 launch nav_rtabmap bringup.launch.py
```

Start autonomous exploration.

```bash
ros2 topic pub /explorer_start std_msgs/msg/Bool "data: true" --once
```

## Run on Real Robot

Open a terminal and source the workspace.

```bash
source ~/ros2_ws/install/setup.bash
```

Run keyboard teleoperation with stamped velocity commands.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

Open another terminal and source the workspace again.

```bash
source ~/ros2_ws/install/setup.bash
```

Launch the real robot system.

```bash
ros2 launch nav_rtabmap bringup_real.launch.py
```

Start autonomous exploration.

```bash
ros2 topic pub /explorer_start std_msgs/msg/Bool "data: true" --once
```

## Notes

Use `bringup.launch.py` for simulation.

Use `bringup_real.launch.py` for the real robot.

Before running on the real robot, make sure the robot is connected and the required topics are available.
