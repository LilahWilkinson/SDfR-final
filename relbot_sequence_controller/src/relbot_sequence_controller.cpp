#include "steering.hpp"
#include <time.h>

SteerRelbot::SteerRelbot() : Node("steer_relbot") {
    RCLCPP_INFO(this->get_logger(), "Init");

    #ifdef HAS_XRF2_MSGS
        xrf2_included_ = true;
    #endif

    // initialize attributes
    left_velocity = 0;
    right_velocity = 0;

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
    // RCLCPP_INFO(this->get_logger(), "Received object position");
}

void SteerRelbot::size_topic_callback(const example_interfaces::msg::Float64::SharedPtr size) {
    object_size = size->data;
    // RCLCPP_INFO(this->get_logger(), "Received object size");
}

void SteerRelbot::calculate_velocity() {
    /* comments from Max:
     *  If object_position is a struct (as I suspect it is), should the
     *      conditionals under 'else if (relative distance > 0)' not be
     *      'object_position->data' instead of just 'object_position'?
     *      This is also the case for the next conditional (< 0).
     */
    double distance_scale = -0.36;
    double distance_offset = 0.27;
    double distance = distance_scale * object_size + distance_offset; // approximate distance in m based on object size relative to FOV size
    RCLCPP_INFO(this->get_logger(), "Approximate distance: %f, object size: %f, object position: %f", distance, object_size, object_position);
    double speed = 5;
    double turn = 1;
    double setpoint_distance = 1;
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
            left_velocity = -relative_distance * speed;    
        }
        else if (object_position > 0) {
            right_velocity = relative_distance * speed;    
            left_velocity = -relative_distance * speed - object_position * turn;
        }
        else {
            right_velocity = relative_distance * speed;
            left_velocity = -relative_distance * speed;
        }
    }
    else if (relative_distance < 0){    // closer than setpoint following distance
        RCLCPP_INFO(this->get_logger(), "Going backward - Approximate distance: %f, relative distance: %f", distance, relative_distance);
        if (object_position < 0) {
            right_velocity = relative_distance * speed - object_position * turn;
            left_velocity = - relative_distance * speed;    
        }
        else if (object_position > 0) {
            right_velocity = relative_distance * speed;    
            left_velocity = - relative_distance * speed - object_position * turn;
        }
        else {
            right_velocity = relative_distance * speed;
            left_velocity = - relative_distance * speed;
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

    // publish velocity to simulator
    example_interfaces::msg::Float64 left_wheel;
    example_interfaces::msg::Float64 right_wheel;
    left_wheel.data = -left_velocity;
    right_wheel.data = right_velocity;
    if (xrf2_included_ == false) {
        RCLCPP_INFO(this->get_logger(), "on real RELbot: invert left wheel velocity");
        left_wheel.data = left_velocity;
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