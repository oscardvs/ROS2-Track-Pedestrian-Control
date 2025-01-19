// control_barrel_world_node.cpp

#include "control_barrel_world/control_barrel_world_node.hpp"
#include <limits>
#include <cmath>

// Constructor
ControlBarrelWorldNode::ControlBarrelWorldNode()
    : Node("control_barrel_world_node")
{
    // Declare parameters with default values for linear and angular speed
    this->declare_parameter<double>("linear_speed", 0.1);
    this->declare_parameter<double>("angular_speed", 0.2);

    // Get parameters values
    linear_speed_ = this->get_parameter("linear_speed").as_double();
    angular_speed_ = this->get_parameter("angular_speed").as_double();

    // Publisher to /mirte/cmd_vel
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/mirte/cmd_vel", 10);

    // Subscribe to obstacle detections from the 3D sensor
    detections_subscriber_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
        "/detections", 10, std::bind(&ControlBarrelWorldNode::detections_callback, this, std::placeholders::_1));

    // Subscribe to pedestrian detections from the camera
    pedestrians_subscriber_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "/pedestrians", 10, std::bind(&ControlBarrelWorldNode::pedestrians_callback, this, std::placeholders::_1));
}

void ControlBarrelWorldNode::detections_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
    // Vector to hold filtered detection centers
    std::vector<geometry_msgs::msg::Point> filtered_points;

    // Process each detection
    for (const auto& detection : msg->detections)
    {
        // Get the center of the bounding box
        const auto& center = detection.bbox.center.position;

        // Calculate distance from the robot to the detection
        double distance = std::sqrt(center.x * center.x + center.y * center.y + center.z * center.z);

        // Filter detections in front of the robot (x > 0) and within 0.7 meters
        if (center.x > 0 && distance <= 0.7)
        {
            filtered_points.push_back(center);
        }
    }

    // Initialize the Twist message for controlling the robot
    auto twist_msg = geometry_msgs::msg::Twist();

    // Only proceed if a person has not been detected yet
    if (!person_detected_)
    {
        if (filtered_points.empty())
        {
            // No obstacles detected; drive forward
            twist_msg.linear.x = linear_speed_;
            twist_msg.angular.z = 0.0;
        }
        else
        {
            // Obstacles detected; get the closest one
            auto closest_point = get_closest_point(filtered_points);
            double y = closest_point.y; // Get the y-coordinate

            twist_msg.linear.x = linear_speed_; // Maintain forward speed

            if (y > 0)
            {
                // Obstacle is to the left; steer right
                twist_msg.angular.z = -angular_speed_;
            }
            else
            {
                // Obstacle is to the right; steer left
                twist_msg.angular.z = angular_speed_;
            }
        }

        // Publish the Twist message
        cmd_vel_publisher_->publish(twist_msg);
    }
    else
    {
        // Person detected; stop the robot
        cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
    }
}

//Callback function that processes pedestrian detections
//This function is called when a new message is received on the /pedestrians topic
void ControlBarrelWorldNode::pedestrians_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    //Loop through each pedestrian detection in the message
    for (const auto& detection : msg->detections)
    {
        //Get the size of bounding box
        float width = detection.bbox.size_x;
        float height = detection.bbox.size_y;
        // Calculate area
        float area = width * height;

	//If the bounding box area is larger than 20000px^2 - > person detected
        if (area > 20000.0)
        {
            // Person detected; stop the robot
            person_detected_ = true;
            // Stop publishing non-zero Twist messages
            cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
            return; // Exit the function early since we've detected a person
        }
    }
}

// Function to find the closest point in a vector of points
geometry_msgs::msg::Point ControlBarrelWorldNode::get_closest_point(const std::vector<geometry_msgs::msg::Point>& points)
{
    // Initialize the minimum distance to a very large value
    double min_distance = std::numeric_limits<double>::max();
    // Variable to store the closest point
    geometry_msgs::msg::Point closest_point;
    
    // Loop through each point in the vector
    for (const auto& point : points)
    {
        // Calculate the Euclidean distance from the origin (robot) to the current point
        double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

        // If this point is closer than the current minimum, update the minimum distance and closest point
        if (distance < min_distance)
        {
            min_distance = distance;// Update the minimum distance
            closest_point = point;// Store the closest point
        }
    }

    return closest_point;
}


	
