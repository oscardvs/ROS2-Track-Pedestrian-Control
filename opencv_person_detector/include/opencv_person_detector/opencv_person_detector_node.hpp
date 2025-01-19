#ifndef OPENCV_PERSON_DETECTOR_NODE_HPP
#define OPENCV_PERSON_DETECTOR_NODE_HPP

// Include necessary ROS2 headers and message types
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

// Include OpenCV headers
#include <opencv2/imgproc.hpp> // Provides image processing functions such as resizing, filtering, edge detection, and color space conversions.
#include <opencv2/highgui.hpp> // Provides functions for creating graphical user interfaces, such as displaying images or creating windows.
#include <opencv2/objdetect.hpp> // Provides object detection functionality, including pre-trained classifiers like the HOG detector and Haar cascades.
 
// Forward declaration of the class
class OpencvPersonDetectorNode : public rclcpp::Node
{
public:
    OpencvPersonDetectorNode();

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    // Member variables
    cv::HOGDescriptor hog_;  // OpenCV HOGDescriptor for person detection

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_publisher_;
};

#endif // OPENCV_PERSON_DETECTOR_NODE_HPP
