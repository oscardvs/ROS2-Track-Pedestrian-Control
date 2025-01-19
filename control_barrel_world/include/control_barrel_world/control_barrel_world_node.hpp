// control_barrel_world_node.hpp

#ifndef CONTROL_BARREL_WORLD_NODE_HPP
#define CONTROL_BARREL_WORLD_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

class ControlBarrelWorldNode : public rclcpp::Node
{
public:
    ControlBarrelWorldNode();

private:
    void detections_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);
    void pedestrians_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    geometry_msgs::msg::Point get_closest_point(const std::vector<geometry_msgs::msg::Point>& points);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detections_subscriber_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr pedestrians_subscriber_;

    double linear_speed_;
    double angular_speed_;
    bool person_detected_ = false;
};

#endif // CONTROL_BARREL_WORLD_NODE_HPP

