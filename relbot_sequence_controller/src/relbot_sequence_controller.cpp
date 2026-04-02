#include "steering.hpp"
#include <time.h>

SteerRelbot::SteerRelbot() : Node("steer_relbot") {
    RCLCPP_INFO(this->get_logger(), "Init");

    // declaring parameters
    // this->declare_parameter("path_", SteerRelbot::DEFAULT_PATH);
    // this->declare_parameter("direction_", SteerRelbot::DEFAULT_DIRECTION);

    // initialize attributes
    left_velocity = 0;
    right_velocity = 0;

    // declaring timer
    start = std::time(0);

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
    object_position = pos;
    // RCLCPP_INFO(this->get_logger(), "Received object position");
}

void SteerRelbot::size_topic_callback(const example_interfaces::msg::Float64::SharedPtr size) {
    object_size = size;
    // RCLCPP_INFO(this->get_logger(), "Received object size");
}

void SteerRelbot::calculate_velocity() {
    double distance_scale = -0.01;
    double distance_offset = 0.75;
    double distance = distance_scale * object_size->data + distance_offset; // approximate distance in m based on object size relative to FOV size
    RCLCPP_INFO(this->get_logger(), "Approximate distance: %f", distance);
    double speed = 3;
    double turn = 1;

    if (distance > 0){
        if (object_position < 0) {
            right_velocity = distance * speed - object_position->data * turn;
            left_velocity = -distance * speed;    
        }
        else if (object_position > 0) {
            right_velocity = distance * speed;    
            left_velocity = -distance * speed - object_position->data * turn;
        }
        else {
            right_velocity = distance * speed;
            left_velocity = -distance * speed;
        }
    }
    else if (distance < 0.5){
        if (object_position < 0) {
            right_velocity = -distance * speed + object_position->data * turn;
            left_velocity = distance * speed;    
        }
        else if (object_position > 0) {
            right_velocity = -distance * speed;    
            left_velocity = distance * speed + object_position->data * turn;
        }
        else {
            right_velocity = -distance * speed;
            left_velocity =  distance * speed;
        }
    }
    else {
        right_velocity = 0;
        left_velocity =  0;
    }

    // RCLCPP_INFO(this->get_logger(), "Velocity: left %f, right %f", left_velocity, right_velocity);
}

void SteerRelbot::timer_callback() {
    // calculate velocity
    calculate_velocity();

    // publish velocity to simulator
    example_interfaces::msg::Float64 left_wheel;
    left_wheel.data = left_velocity;
    example_interfaces::msg::Float64 right_wheel;
    right_wheel.data = right_velocity;
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