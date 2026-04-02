Package relbot_sequence_controller
-----------------------------------------------
#### Description: 
This package contains a skeleton node that you will have to supplement with an algorithm.


#### Output:
`/input/left_motor/setpoint_vel`
- Type: example_interfaces/msg/Float64

`/input/left_motor/setpoint_vel`
- Type: example_interfaces/msg/Float64

#### Run:
To start the node run the following command:

`ros2 run relbot_sequence_controller steer_robot`

You can also launch this node together with turtlesim, the relbot2turtlesim node and the RELbot simulator. Make sure that all nodes are build and sourced in your terminal.

To launch the node, together with the RELbot simulator, relbot2turtlesim and turtlesim run the following command:

`ros2 launch relbot_launch relbot_sequence_controller.launch.py`

#### Parameters:
-

#### Core components:
- calculate_velocity(): The function that you will have to alter.
- timer_callback(): publishes the velocity to both wheel 30 times per second.