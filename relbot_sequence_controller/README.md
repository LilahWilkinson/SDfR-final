Package relbot_sequence_controller
-----------------------------------------------
#### Description: 
This package contains a node that receives information from image_processing (tracked object relative position and size), calculates appropriate left and right wheel velocities to follow the object, and publishes these for use by either the real RELbot or simulator.

#### Input:
`/image_processing/object_position`
- Type: example_interfaces/msg/Float64

`/image_processing/object_size`
- Type: example_interfaces/msg/Float64

#### Output:
`/input/left_motor/setpoint_vel`
- Type: example_interfaces/msg/Float64

`/input/left_motor/setpoint_vel`
- Type: example_interfaces/msg/Float64

#### Run:
To start the node run the following command:

`ros2 run relbot_sequence_controller steer_robot` or `ros2 run relbot_sequence_controller relbot_sequence_controller`

You can also launch this node together with turtlesim, the relbot2turtlesim node, the RELbot simulator, the RELbot adapter and the image_processing node. Make sure that all nodes are built and sourced in your terminal.

To launch the node, together with the RELbot simulator, relbot2turtlesim and turtlesim run the following command:

`ros2 launch relbot_launch relbot_sequence_controller.launch.py`

Testing full pipeline with simulated RELbot: to launch the node, together with the RELbot simulator, relbot2turtlesim, turtlesim, RELbot adapter and image processing node, do the following:

- change the variable CAMERA_IMAGE in image_processing/image_processing.hpp to `/output/moving_camera`
- change the variable DEFAULT_ROBOT_MODE in relbot_adapter/relbot_adapter.hpp to `sim`
- change the variable DEFAULT_ROBOT_MODE in relbot_sequence_controller/steering.hpp to `sim`
- run `ros2 launch relbot_launch relbot_simulated_system_final.launch.py`

Testing full pipeline with real RELbot: to launch the node, together with the RELbot adapter and image processing node, do the following:

- change the variable CAMERA_IMAGE in image_processing.hpp to `/image`
- change the variable DEFAULT_ROBOT_MODE in relbot_adapter/relbot_adapter.hpp to `real`
- change the variable DEFAULT_ROBOT_MODE in relbot_sequence_controller/steering.hpp to `real`
- run `ros2 launch relbot_launch relbot_real_system_final.launch.py`

#### Core components:
- calculate_velocity(): calculates estimated distance (m) of object based on relative size. Calculates appropriate wheel velocities (rad/s) proportional to distance, with left/right variations proportional to object x-position relative to image center.
- position_topic_callback(), size_topic_callback(): update current position and size attributes whenever topic updates
- timer_callback(): publishes the velocity to both wheels 30 times per second.