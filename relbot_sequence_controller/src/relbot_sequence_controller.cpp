#include "steering.hpp"
#include <time.h>

SteerRelbot::SteerRelbot() : Node("steer_relbot") {
    RCLCPP_INFO(this->get_logger(), "Init");

    // initialize attributes
    left_velocity = 0;
    right_velocity = 0;
    signal_interval_counter = 0;    // used to determine time since last received signal

    // initialize topics
    create_topics();
    RCLCPP_INFO(this->get_logger(), "Created Topics");

    // initialize timer
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1/DEFAULT_SETPOINT_STREAM),
                                     std::bind(&SteerRelbot::timer_callback, this));
}

/**
 * @brief Create all topics for this node. Two subscriptions for the tracking object position and
 * size, and two publishers for desired left and right wheel velocities.
 */
void SteerRelbot::create_topics() {
    // creating publishers
    left_wheel_topic_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/input/left_motor/setpoint_vel", 1);

    right_wheel_topic_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/input/right_motor/setpoint_vel", 1);

    // creating subscriptions
    RCLCPP_INFO(this->get_logger(), "Creating Subscriptions");
    object_position_topic_ = this->create_subscription<example_interfaces::msg::Float64>(
        SteerRelbot::OBJECT_POSITION, 10, std::bind(&SteerRelbot::position_topic_callback, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "Subscribed to topic: /image_processing/object_position");
    object_size_topic_ = this->create_subscription<example_interfaces::msg::Float64>(
        SteerRelbot::OBJECT_SIZE, 10, std::bind(&SteerRelbot::size_topic_callback, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "Subscribed to topic: /image_processing/object_size");
}

/**
 * @brief Store new tracking object position as class attribute when the topic is updated. Reset
 * counter for signal intervals.
 * 
 * @param pos object position relative to FOV width and x-center
 */
void SteerRelbot::position_topic_callback(const example_interfaces::msg::Float64::SharedPtr pos) {
    object_position = pos->data;
    signal_interval_counter = 0; // reset counter of velocity callbacks since last position update
}

/**
 * @brief Store new tracking object size as class attribute when the topic is updated. Reset
 * counter for signal intervals.
 * 
 * @param pos object size relative to FOV size
 */
void SteerRelbot::size_topic_callback(const example_interfaces::msg::Float64::SharedPtr size) {
    object_size = size->data;
    signal_interval_counter = 0; // reset counter of velocity callbacks since last size update
}

/**
 * @brief Calculate velocity proportional to tracking object position and size. The function
 * uses an estimate of the object distance in m and a pre-set following distance to get
 * relative distance. If relative dist. > 0, the robot moves forward. If the object is to the 
 * right of the image center, the robot turns right, and vice versa. If relative dist. < 0,
 * the same is done but backwards.
 */
void SteerRelbot::calculate_velocity() {
    // calculate approximate distance of tracking object to camera
    double distance = estimate_distance();
    
    // set velocity characteristics
    double speed = 10;              // rad/s per m distance
    double turn = 3;                // rad/s per x-offset unit in FOV
    double setpoint_distance = 0.3; // target following distance in m
    double relative_distance = distance - setpoint_distance;

    // calculate appropriate velocity
    if (object_size < 0.0001) { // if no object is detected
        RCLCPP_INFO(this->get_logger(), "Object size too small");
        right_velocity = 0;
        left_velocity =  0;
    }
    else if (relative_distance > 0){    // object distance farther than setpoint following distance
        RCLCPP_INFO(this->get_logger(), "Going forward - Approx. dist: %f, dist to setpoint: %f, object size: %f, object position: %f, ", distance, relative_distance, object_size, object_position);
        if (object_position < 0) {      // object to the left of image center
            // base velocity proportional to relative distance, added turning velocity proportional to x-offset
            right_velocity = relative_distance * speed - object_position * turn;    // turn left
            left_velocity = relative_distance * speed;    
        }
        else if (object_position > 0) { // object to the right of image center
            // base velocity proportional to relative distance, added turning velocity proportional to x-offset
            right_velocity = relative_distance * speed;    
            left_velocity = relative_distance * speed + object_position * turn;     // turn right
        }
        else {                          // object at image center
            // base velocity proportional to relative distance, go straight ahead
            right_velocity = relative_distance * speed;
            left_velocity = relative_distance * speed;
        }
    }
    else if (relative_distance < 0){    // object distance closer than setpoint following distance
        RCLCPP_INFO(this->get_logger(), "Going backward - Approx. dist: %f, dist to setpoint: %f, object size: %f, object position: %f, ", distance, relative_distance, object_size, object_position);
        if (object_position < 0) {      // object to the left of image center
            // base velocity proportional to relative distance, added turning velocity proportional to x-offset
            right_velocity = relative_distance * speed - object_position * turn;    // turn 'left'
            left_velocity = relative_distance * speed;    
        }
        else if (object_position > 0) { // object to the right of image center
            // base velocity proportional to relative distance, added turning velocity proportional to x-offset
            right_velocity = relative_distance * speed;    
            left_velocity = relative_distance * speed + object_position * turn;     // turn 'right'
        }
        else {
            // base velocity proportional to relative distance, go straight backwards
            right_velocity = relative_distance * speed;
            left_velocity = relative_distance * speed;
        }
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Doing nothing - else condition");
        right_velocity = 0;
        left_velocity =  0;
    }
}

/**
 * @brief Estimate distance based on logarithmic 
 */
double SteerRelbot::estimate_distance() {
    double distance_scale = -0.1558;
    double distance_offset = -0.1198;

    // calculate approximate distance in m based on object size relative to FOV size
    double distance = distance_scale * std::log(object_size) + distance_offset; 
    return distance;
}

void SteerRelbot::timer_callback() {
    // calculate velocity
    calculate_velocity();

    // check how many callbacks have passed since last position/size update. If more than 2 seconds, set velocity to 0
    signal_interval_counter = signal_interval_counter + 1;
    if (signal_interval_counter >= int(2 * DEFAULT_SETPOINT_STREAM)) {
        left_velocity = 0;
        right_velocity = 0;
    }

    // publish velocity to simulator
    left_wheel.data = left_velocity;
    right_wheel.data = right_velocity;
    // left wheel velociy is inverted on the simulator
    if (DEFAULT_ROBOT_MODE == "sim") {
        left_wheel.data = -left_velocity;
    }
    RCLCPP_INFO(this->get_logger(), "Velocity: left %f, right %f", left_velocity, right_velocity);
    left_wheel_topic_->publish(left_wheel);
    right_wheel_topic_->publish(right_wheel);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SteerRelbot>());
    rclcpp::shutdown();
    return 0;
}