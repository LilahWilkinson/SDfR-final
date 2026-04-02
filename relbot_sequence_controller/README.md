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
path_: choses the path the robot will follow. Initially set to do nothing. Value can be 1-5 (any other will set the robot to be motionless)
    1: straight line
    2: circle
    3: straight line with 90 degree left turn
    4: square with rounded corners
    5: user input (see direction_ for how to control)
    Set with : `ros2 param set /relbot_sequence_controller path_ <value>`
direction_ : choses the direction when the path is set to user input. Initially set to do nothing. Value can be 6-9 (any other can be used to stop the robot from moving)
    6: straight
    7: left turn
    8: backward
    9: right turn
    Set with : `ros2 param set /relbot_sequence_controller direction_ <value>`

#### Core components:
- calculate_velocity(): Function that allows the user to chose between 5 settings: 4 prescribed paths and one that allows for user inputs.
- timer_callback(): publishes the velocity to both wheel 30 times per second.