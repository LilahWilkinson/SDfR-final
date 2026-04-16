//==================================================================================================
// Authors : I.M. Kramers & L.S. Wilkinson
// Group : 14
// License : LGPL open source license
//
// Brief : image_processing receives camera output from relbot_simulator or cam2image_vm2ros. Each
// frame is processed in OpenCV HSV format to detect a green object, and calculate its size relative
// to the FOV size as well as position relative to FOV x-center. These values are published to
// separate channels. For monitoring purposes, the processed image is also published.
//==================================================================================================

#ifndef IMAGE_PROCESSING_HPP_
#define IMAGE_PROCESSING_HPP_

// CPP library headers
#include <cstdio>
#include <chrono>
#include <memory>

// ROS Client Library CPP
#include "rclcpp/rclcpp.hpp"

// Standard Message types
#include "example_interfaces/msg/float64.hpp"

// Sensor message types related to images
#include "sensor_msgs/msg/image.hpp"

// Image processing functions
#include "image_functions_sdfr/image_functions.hpp"

// OpenCV image processing functions
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

#if __has_include(<cv_bridge/cv_bridge.hpp>)
    #include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
    #include <cv_bridge/cv_bridge.h>
#else
    #error "Required cv_bridge header file not found"
#endif

using std::placeholders::_1;
using namespace std::chrono_literals;

class ImageProcessor : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new image processor object
     */
    ImageProcessor();

    // topic name to subscribe to (/image for real RELbot and /output/moving_camera for simulator)
    const std::string CAMERA_IMAGE = "/image";

private:
    // topics
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_input_topic_;           // input image
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr object_position_topic_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr object_size_topic_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_output_topic_;             // publish processed image for monitoring

    // received image
    sensor_msgs::msg::Image::SharedPtr input_image;

    // HSV tresholds to identify a certain color (green)
    const int min_hue = 35;                     // yellow side
    const int max_hue = 95;                     // blue side

    const int min_saturation = 45;              // 0 = gray
    const int max_saturation = 255;

    const int min_value = 55;                   // 0 = black
    const int max_value = 255;

    // object information
    int image_width;                            // image width in pixels
    int image_height;                           // image height in pixels
    example_interfaces::msg::Float64 position;  // tracking object x-position relative to image center and FOV size
    example_interfaces::msg::Float64 size;      // estimated tracking object size relative to camera area


    /**
     * @brief Create all topics for this node. One subscription for the webcam input,
     * and three publishers for the processed image, and object position and size.
     */
    void create_topics();

    /**
     * @brief Handles receiving of received webcam images. Callback upon receiving an 
     * image from the simulator or RELbot. Finds the relative size of the tracking object 
     * and relative position to horizontal image center, and publishes those.
     *
     * @param msg_cam_img webcam image, in sensor_msgs/msg/Image format
     */
    void camera_topic_callback(const sensor_msgs::msg::Image::SharedPtr msg_cam_img);

    /**
     * @brief Image processing to detect a green object in the image. Method based on 
     * https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html.
     * Image is first converted to OpenCV HSV format. Each pixel is checked against the 
     * target HSV range. A binary mask is created with all accepted pixels. Morphological 
     * opening and closing is used to remove small objects and holes. Object moments are 
     * calculated to get relative object x-position. Relative size is calculated from the 
     * object pixel count and total picture area. Finally, the processed image is published 
     * for monitoring purposes.
     */
    void process_image();

    /**
     * @brief Publishes the tracking object position relative to FOV center and width,
     * as type example_interfaces/msg/Float64.
     */
    void publish_obj_position();

    /**
     * @brief Publishes the tracking object size relative to FOV size, as type 
     * example_interfaces/msg/Float64.
     */
    void publish_obj_size();

    /**
     * @brief Converts the tresholded and morphed image to sensor_msgs/msg/Image format,
     * then publishes it.
     * 
     * @param image processed image, in cv::Mat format
     */
    void publish_processed_image(cv::Mat image);
};

#endif