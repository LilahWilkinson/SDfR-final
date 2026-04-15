#include "steering.hpp"
#include <time.h>

SteerRelbot::SteerRelbot() : Node("steer_relbot") {
    RCLCPP_INFO(this->get_logger(), "Init");

    // #ifdef HAS_XRF2_MSGS
    //     xrf2_included_ = true;
    // #endif

    // initialize attributes
    left_velocity = 0;
    right_velocity = 0;
    signal_interval_counter = 0;

    // initialize topics
    create_topics();
    RCLCPP_INFO(this->get_logger(), "Created Topics");

    // initialize timer
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1/DEFAULT_SETPOINT_STREAM),
                                     std::bind(&SteerRelbot::timer_callback, this));
}

void SteerRelbot::create_topics() {
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

void SteerRelbot::position_topic_callback(const example_interfaces::msg::Float64::SharedPtr pos) {
    object_position = pos->data;
    signal_interval_counter = 0; // reset counter of velocity callbacks since last position update
    // RCLCPP_INFO(this->get_logger(), "Received object position");
}

void SteerRelbot::size_topic_callback(const example_interfaces::msg::Float64::SharedPtr size) {
    object_size = size->data;
    signal_interval_counter = 0; // reset counter of velocity callbacks since last size update
    // RCLCPP_INFO(this->get_logger(), "Received object size");
}

void SteerRelbot::calculate_velocity() {
    double distance_scale = -0.1558;
    double distance_offset = -0.1198;
    double distance = distance_scale*std::log(object_size)+distance_offset; // approximate distance in m based on object size relative to FOV size
    RCLCPP_INFO(this->get_logger(), "Approximate distance: %f, object size: %f, object position: %f", distance, object_size, object_position);
    double speed = 10;
    double turn = 3;
    double setpoint_distance = 0.3;
    double relative_distance = distance - setpoint_distance;

    if (object_size < 0.0001) { // if no object is detected
        RCLCPP_INFO(this->get_logger(), "object size too small");
        right_velocity = 0;
        left_velocity =  0;
    }
    else if (relative_distance > 0){    // farther than setpoint following distance
        RCLCPP_INFO(this->get_logger(), "Going forward - Approximate distance: %f, relative distance: %f", distance, relative_distance);
        if (object_position < 0) {
            right_velocity = relative_distance * speed - object_position * turn;
            left_velocity = relative_distance * speed;    
        }
        else if (object_position > 0) {
            right_velocity = relative_distance * speed;    
            left_velocity = relative_distance * speed + object_position * turn;
        }
        else {
            right_velocity = relative_distance * speed;
            left_velocity = relative_distance * speed;
        }
    }
    else if (relative_distance < 0){    // closer than setpoint following distance
        RCLCPP_INFO(this->get_logger(), "Going backward - Approximate distance: %f, relative distance: %f", distance, relative_distance);
        if (object_position < 0) {
            right_velocity = relative_distance * speed + object_position * turn;
            left_velocity = relative_distance * speed;    
        }
        else if (object_position > 0) {
            right_velocity = relative_distance * speed;    
            left_velocity = relative_distance * speed - object_position * turn;
        }
        else {
            right_velocity = relative_distance * speed;
            left_velocity = relative_distance * speed;
        }
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Doing nothing - else condition");
        right_velocity = 0;
        left_velocity =  0;
    }

    // RCLCPP_INFO(this->get_logger(), "Velocity: left %f, right %f", left_velocity, right_velocity);
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
    // if (xrf2_included_ == false) {
    //     RCLCPP_INFO(this->get_logger(), "on simulated RELbot: invert left wheel velocity");
    //     left_wheel.data = left_velocity;
    // }
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