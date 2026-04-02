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
    // const int DEFAULT_PATH = 5;      
    // const int DEFAULT_DIRECTION = 0;

private:
    // Topics
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr object_position_topic_;
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr object_size_topic_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_wheel_topic_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_wheel_topic_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // attributes
    std::shared_ptr<example_interfaces::msg::Float64> object_position;
    std::shared_ptr<example_interfaces::msg::Float64> object_size;
    double left_velocity;
    double right_velocity;
    std::time_t start;

    // int path_ = DEFAULT_PATH;
    // int direction_ = DEFAULT_DIRECTION;

    // methods
    void create_topics();
    void position_topic_callback(const example_interfaces::msg::Float64::SharedPtr pos);
    void size_topic_callback(const example_interfaces::msg::Float64::SharedPtr size);
    void timer_callback();
    void calculate_velocity();
};

#endif