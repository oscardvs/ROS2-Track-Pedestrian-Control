//opencv_person_detector_node.cpp

#include "opencv_person_detector/opencv_person_detector_node.hpp" // Include the header file for the custom node implementation.
#include <cv_bridge/cv_bridge.h> // Allows conversion between ROS Image messages and OpenCV images.
#include <vision_msgs/msg/pose2_d.hpp>  // Defines 2D pose message type for vision tasks.
#include <vision_msgs/msg/point2_d.hpp> // Defines 2D point message type for representing positions in 2D space.

// Constructor
OpencvPersonDetectorNode::OpencvPersonDetectorNode()
	: Node("opencv_person_detector_node")// Initialize
{
	// Subscriber for camera images
	// 'this' refers to the current instance of OpencvPersonDetectorNode, allowing access to its member functions and variables.
	image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("/mirte/camera_rgb/image_raw", 10, std::bind(&OpencvPersonDetectorNode::image_callback, this, std::placeholders::_1));
	// bind - Used to bind the member function image_callback to the callback for the subscription. Since image_callback is a member of the class, we need to pass the current object (this) to associate the callback with this instance of OpencvPersonDetectorNode.
	
        // Publisher for visualization of the processed image
	image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/visual", 10);

	// Publisher for detected persons
	detection_publisher_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("/pedestrians", 10);

	// Initialize HOGDescriptor with the default people detector
	hog_ = cv::HOGDescriptor();
	hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
}

// Callback function for image processing
void OpencvPersonDetectorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	// Convert the ROS Image message to OpenCV format
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		// Convert the image to BGR8 format
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	}
        catch (cv_bridge::Exception& e)
	{
		// Log an error if the conversion fails
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
 		return;
	}

	// Detect people in the image using HOGDescriptor
	std::vector<cv::Rect> detections;
	hog_.detectMultiScale(cv_ptr->image, detections, 0, cv::Size(8, 8));

	// Prepare the Detection2DArray message
	vision_msgs::msg::Detection2DArray detection_msg;
	detection_msg.header = msg->header; // Set the header of the message

	// Process each detection
	for (const auto& detection : detections)
	{
		// Draw red rectangle around detected person
		cv::rectangle(cv_ptr->image, detection, cv::Scalar(0, 0, 255), 2);

		// Create a Detection2D message
		vision_msgs::msg::Detection2D detection_2d;

		// Set the center position
		detection_2d.bbox.center.position.x = detection.x + detection.width / 2.0;
		detection_2d.bbox.center.position.y = detection.y + detection.height / 2.0;
			
		// Set the orientation of the bounding box
		detection_2d.bbox.center.theta = 0.0;
			
		// Set the size of the bounding box
		detection_2d.bbox.size_x = detection.width;
		detection_2d.bbox.size_y = detection.height;

		// Add the detection to the array
		detection_msg.detections.push_back(detection_2d);
	}

	// Publish the detection results to the /pedestrians topic
	detection_publisher_->publish(detection_msg);

	// Convert the modified OpenCV image back to ROS message and publish
	cv_bridge::CvImage out_msg;
	out_msg.header = cv_ptr->header;// Keep the same header
	out_msg.encoding = sensor_msgs::image_encodings::BGR8;
	out_msg.image = cv_ptr->image;

	// Publish the image with visualized detections to the /visual topic
	image_publisher_->publish(*out_msg.toImageMsg());
}

