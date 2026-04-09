#include "image_processing.hpp"

ImageProcessor::ImageProcessor() : Node("image_processing_node") {
    RCLCPP_INFO(this->get_logger(), "Init");

    // initialize object data variables to 0
    image_width = 0;
    image_height = 0;
    past_size_frac = 1;
    position.data = 0;
    size.data = 0;

    // initialize topics
    create_topics();
    RCLCPP_INFO(this->get_logger(), "Created Topics");
}

void ImageProcessor::create_topics() {
    RCLCPP_INFO(this->get_logger(), "Creating topics...");

    RCLCPP_INFO(this->get_logger(), "Creating Publishers");
    object_position_topic_ = this->create_publisher<example_interfaces::msg::Float64>("/image_processing/object_position", 1);
    object_size_topic_ = this->create_publisher<example_interfaces::msg::Float64>("/image_processing/object_size", 1);
    camera_output_topic_ = this->create_publisher<sensor_msgs::msg::Image>("/image_processing/processed_image", 1);

    RCLCPP_INFO(this->get_logger(), "Creating Subscriptions");
    camera_input_topic_ = this->create_subscription<sensor_msgs::msg::Image>(
        ImageProcessor::CAMERA_IMAGE, 10, std::bind(&ImageProcessor::camera_topic_callback, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "Subscribed to topic: /image"); // for simulator: /output/moving_camera
}

/**
 * @brief Handles receiving of received webcam images
 *
 * @param msg_cam_img webcam image, in sensor_msgs::msg::Image format
 */
void ImageProcessor::camera_topic_callback(const sensor_msgs::msg::Image::SharedPtr msg_cam_img) {
    input_image = msg_cam_img;
    // RCLCPP_INFO(this->get_logger(), "Received image...");

    // check that image is not empty
    if (input_image == nullptr) {
        RCLCPP_INFO(this->get_logger(), "WARNING: Empty image");
        return;
    }
    else {
        process_image();
        publish_obj_position();
        publish_obj_size();
    }
};

/**
 * @brief Image processing. Method based on https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html
 */
void ImageProcessor::process_image() {
    // get image dimensions
    image_width = image_functions::getImageWidth(input_image);
    image_height = image_functions::getImageHeight(input_image);

    // copy image to cv-readable format, convert from BGR to HSV
    cv::Mat cv_image = cv_bridge::toCvCopy(input_image, "bgr8" /* or bgra8, rgb8, rgba8, mono8, mono16 */)->image;
    cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2HSV);

    // treshold image based on manually defined HSV range for green
    cv::Mat cv_image_processed;
    cv::inRange(cv_image, cv::Scalar(min_hue, min_saturation, min_value), cv::Scalar(max_hue, max_saturation, max_value), cv_image_processed);

    // morphological opening and closing to remove small objects from binary image and fill small holes in the detected object
    cv::morphologyEx(cv_image_processed, cv_image_processed, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9)));
    cv::morphologyEx(cv_image_processed, cv_image_processed, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9)));

    // finding moments
    cv::Moments obj_moments = cv::moments(cv_image_processed);

    double moment_10 = obj_moments.m10 / 255;
    double area = obj_moments.m00 / 255;

    if (area > 25) {
        //calculate the position of the ball
        int obj_x_position = moment_10 / area;

        int half_image_width = int(image_width/2);
        int x_pos_from_center = int(obj_x_position - half_image_width);
        double x_percent = double(x_pos_from_center)/double(half_image_width);
        RCLCPP_INFO(this->get_logger(), "x_percent: %f", x_percent);

        double total_area = image_width*image_height;
        double size_frac = area/total_area;

        position.data = x_percent;
        size.data = size_frac;
    }
    else {
        position.data = 0;
        size.data = 0;
    }

    // publish processed image to check that the object is identified correctly
    cv::cvtColor(cv_image_processed, cv_image_processed, cv::COLOR_GRAY2BGR);
    publish_processed_image(cv_image_processed);
}

void ImageProcessor::publish_obj_position() {
    object_position_topic_->publish(position);
}

void ImageProcessor::publish_obj_size() {
    object_size_topic_->publish(size);
}

void ImageProcessor::publish_processed_image(cv::Mat image) {

    // transform img back to sensor_msg
    cv_bridge::CvImage out_msg;
    // out_msg.header.stamp = get_clock()->now();
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = image;

    std::shared_ptr<sensor_msgs::msg::Image> sensor_output_img = out_msg.toImageMsg();

    camera_output_topic_->publish(*sensor_output_img.get());
}

int main(int argc, char * argv[])
{
  printf("Initialising image processing node\n");
  rclcpp::init(argc, argv);
  printf("Start spinning...\n");
  rclcpp::spin(std::make_shared<ImageProcessor>());
  printf("Shutting down image processing node\n");
  rclcpp::shutdown();
  return 0;
}