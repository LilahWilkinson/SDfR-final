//==================================================================================================
// Authors : I.M. Kramers & L.S. Wilkinson
// Group : 14
// License : LGPL open source license
//
// Brief : This package contains a node that receives information from image_processing (tracked 
// object relative position and size), calculates appropriate left and right wheel velocities to 
// follow the object at a pre-set distance, and publishes these velocities for use by either the 
// real RELbot or simulator. If the image_processing node stops updating the object size and 
// position, output velocity will be set to 0 after 2 seconds.
//==================================================================================================

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
    /**
     * @brief Create all topics for this node. Two subscriptions for the tracking object position and
     * size, and two publishers for desired left and right wheel velocities.
     */
    void create_topics();

    /**
     * @brief Store new tracking object position as class attribute when the topic is updated. Reset
     * counter for signal intervals.
     * 
     * @param pos object position relative to FOV width and x-center
     */
    void position_topic_callback(const example_interfaces::msg::Float64::SharedPtr pos);

    /**
     * @brief Store new tracking object size as class attribute when the topic is updated. Reset
     * counter for signal intervals.
     * 
     * @param size object size relative to FOV size
     */
    void size_topic_callback(const example_interfaces::msg::Float64::SharedPtr size);

    /**
     * @brief Calculate velocity proportional to tracking object position and size. The function
     * uses an estimate of the object distance in m and a pre-set following distance to get
     * relative distance. If relative dist. > 0, the robot moves forward. If the object is to the 
     * right of the image center, the robot turns right, and vice versa. If relative dist. < 0,
     * the same is done but backwards.
     */
    void calculate_velocity();

    /**
     * @brief Estimate distance in m using logarithmic fit (y = a * ln(bx)) of experimentally 
     * determined size vs. distance values. 
     * 
     * @returns estimated object distance in m
     */
    double estimate_distance();

    /**
     * @brief periodically call calculate_velocity() and publish the desired wheel velocities. This 
     * ensures a steady wheel velocity output to the simulator or RELbot. As a failsafe, this function 
     * also checks whether the object size and position have been updated within the last 2 seconds. 
     * If not, the velocity output will be set to 0 until new object data arrives.
     */
    void timer_callback();
    
};

#endif /*STEER_RELBOT_HPP_*/