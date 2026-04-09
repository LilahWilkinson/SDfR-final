/** - Use the input of the light-position indicator, /moving_camera_output of the RELbot simulator 
 *      (just as later in the real case, where the RELbot moves during operation and thus let the webcam 
 *      see moving images of the world). 
 * - Let the NUD produce its outputs in a pace comparable to the pace of its input stream from the webcam, 
 *      so 30 FPS.
 * - Make a launchfile for this test. This makes inspecting your work while grading doable.
 * - 
 */ 

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

// OpenCV imshow
// #include "opencv2/core/mat.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
// #include "opencv2/videoio.hpp"

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

//   const std::string input = "/input";
//   const std::string output = "/output";
    const std::string CAMERA_IMAGE = "/image";  // topic name: relbot_simulator publishes to this topic. For simulator: /output/moving_camera

private:
    // topics
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_input_topic_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr object_position_topic_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr object_size_topic_;
    // publish processed image (binary): used to check whether the object is identified correctly
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_output_topic_; 

    // received image
    sensor_msgs::msg::Image::SharedPtr input_image;

    // HSV tresholds to identify a certain color (green)
    const int min_hue = 35; // 75 red side
    const int max_hue = 95; // 155 blue side (09/04: 85)

    // accept 
    const int min_saturation = 45; // 30
    const int max_saturation = 255;

    const int min_value = 55; // 30 (09/04: 65)
    const int max_value = 255;

    // object information
    int image_width;                            // image width in pixels
    int image_height;                           // image height in pixels
    double past_size_frac;
    example_interfaces::msg::Float64 position;  // tracking object x-position relative to image center and FOV size
    example_interfaces::msg::Float64 size;      // estimated tracking object size relative to camera area


    /**
     * @brief Create all topics for this node
     */
    void create_topics();

    /**
     * @brief Handles receiving of received webcam images. Callback upon receiving an 
     * image from the simulator. Finds the estimated tracking object distance and 
     * relative position, and publishes those.
     *
     * @param msg_cam_img webcam image, in sensor_msgs::msg::Image format
     */
    void camera_topic_callback(const sensor_msgs::msg::Image::SharedPtr msg_cam_img);

    /**
     * @brief Image processing
     */
    void process_image();

    /**
     * @brief Publishes the tracking object position relative to FOV center and width
     */
    void publish_obj_position();

    /**
     * @brief Publishes the tracking object size relative to FOV size
     */
    void publish_obj_size();

    /**
     * @brief Publishes the tresholded and morphed image
     */
    void publish_processed_image(cv::Mat image);
};

#endif