#ifndef STEER_RELBOT_HPP_
#define STEER_RELBOT_HPP_

// CPP library headers

// ROS Client Library CPP
#include "rclcpp/rclcpp.hpp"

// message type for velocity
#include "example_interfaces/msg/float64.hpp"

using std::placeholders::_1;

class SteerRelbot : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new steering object
     */
    SteerRelbot();

    const double DEFAULT_SETPOINT_STREAM = 30;  // How often the velocities are published per second
    const std::string OBJECT_POSITION = "/image_processing/object_position";
    const std::string OBJECT_SIZE = "/image_processing/object_size";

private:
    // Topics
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr object_position_topic_;
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr object_size_topic_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_wheel_topic_;           // desired left wheel velocity
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_wheel_topic_;          // desired right wheel velocity

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    int signal_interval_counter; // used to check if new signals are still coming in

    // attributes
    double object_position;                         // tracking object x-position relative to FOV width and center
    double object_size;                             // tracking object size relative to FOV size
    double left_velocity;                           // desired left wheel velocity (rad/s)
    double right_velocity;                          // desired right wheel velocity (rad/s)
    example_interfaces::msg::Float64 left_wheel;    // desired left wheel velocity (publishable format)
    example_interfaces::msg::Float64 right_wheel;   // desired right wheel velocity (publishable format)

    const std::string DEFAULT_ROBOT_MODE = "real";  // "real" for real RELbot, change to "sim" for testing on computer!

    // methods
    void create_topics();
    void position_topic_callback(const example_interfaces::msg::Float64::SharedPtr pos);
    void size_topic_callback(const example_interfaces::msg::Float64::SharedPtr size);
    void calculate_velocity();
    double estimate_distance();
    void timer_callback();
    
};

#endif /*STEER_RELBOT_HPP_*/