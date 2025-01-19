//opencv_person_detector_node.cpp

#include "opencv_person_detector/opencv_person_detector_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create the node and spin it to process callbacks
    rclcpp::spin(std::make_shared<OpencvPersonDetectorNode>());

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}
