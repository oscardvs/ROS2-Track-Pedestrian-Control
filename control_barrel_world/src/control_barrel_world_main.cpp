// control_barrel_world_main.cpp

#include "control_barrel_world/control_barrel_world_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    // Initialize the ROS2 system
    rclcpp::init(argc, argv);

    // Create the node and spin it to process callbacks
    rclcpp::spin(std::make_shared<ControlBarrelWorldNode>());

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}
