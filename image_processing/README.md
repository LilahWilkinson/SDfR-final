Package relbot_sequence_controller
-----------------------------------------------
#### Description: 
This package contains a node that receives images from either the relbot_simulator node or from the cam2image_vm2ros node (when running the code with the real RELbot). It uses OpenCV to detect a green object in the image, and calculates this object's center point relative to the image x-center as well as the relative size. Both values are published for use by the relbot_sequence_controller node. For optimizing and monitoring purposes, the processed image is also published to a separate topic.

#### Input:
`/output/moving_camera` (simulated RELbot) or `/image` (real RELbot)
- Type: sensor_msgs/msg/Image

#### Output:
`/image_processing/object_position`
- Type: example_interfaces/msg/Float64

`/image_processing/object_size`
- Type: example_interfaces/msg/Float64

`/image_processing/processed_image`
- Type: sensor_msgs/msg/Image

#### Run:
To start the node run the following command:

`ros2 run image_processing image_processing_node`

You can also launch this node together with turtlesim, the relbot2turtlesim node, the RELbot simulator, the RELbot adapter and the RELbot sequence controller. Make sure that all nodes are built and sourced in your terminal.

Testing full pipeline with simulated RELbot: to launch the node, together with the RELbot simulator, relbot2turtlesim, turtlesim, RELbot adapter and RELbot sequence controller, do the following:

- change the variable CAMERA_IMAGE in image_processing/image_processing.hpp to `/output/moving_camera`
- change the variable DEFAULT_ROBOT_MODE in relbot_adapter/relbot_adapter.hpp to `sim`
- change the variable DEFAULT_ROBOT_MODE in relbot_sequence_controller/steering.hpp to `sim`
- run `ros2 launch relbot_launch relbot_simulated_system_final.launch.py`

Testing full pipeline with real RELbot: to launch the node, together with the RELbot adapter and RELbot sequence controller, do the following:

- change the variable CAMERA_IMAGE in image_processing.hpp to `/image`
- change the variable DEFAULT_ROBOT_MODE in relbot_adapter/relbot_adapter.hpp to `real`
- change the variable DEFAULT_ROBOT_MODE in relbot_sequence_controller/steering.hpp to `real`
- run `ros2 launch relbot_launch relbot_real_system_final.launch.py`

#### Core components:
- camera_topic_callback(): when a new image is published, save image and call process_image(). Publish size and position.
- process_image(): convert to OpenCV HSV image. Detect all pixels within certain HSV ranges, create binary mask, open and close to remove small spots and gaps, calculate remaining mask's moments, size and position.
- publish_obj_position(), publish_obj_size() and publish_processed_image(): publish processed values to dedicated topics