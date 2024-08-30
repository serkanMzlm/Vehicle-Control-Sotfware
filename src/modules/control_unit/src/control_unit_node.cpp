#include "control_unit_node.hpp"

using namespace std::placeholders;

ControlUnitNode::ControlUnitNode()
{
    node_ = std::make_shared<rclcpp::Node>("control_unit_node");
    declareParameters();
    controlSelection();
}

void ControlUnitNode::declareParameters()
{
    node_->declare_parameter<std::string>("control_unit", "keyboard");
    node_->declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
    node_->declare_parameter<double>("max_angular_velocity", 1.0);
    node_->declare_parameter<double>("max_linear_velocity", 1.0);

    params.dev_name = node_->get_parameter("device_name").as_string();
    params.control_unit = node_->get_parameter("control_unit").as_string();
    params.angular_limit = node_->get_parameter("max_angular_velocity").as_double(); 
    params.linear_limit  = node_->get_parameter("max_linear_velocity").as_double();
    printDisplay();   
}

void ControlUnitNode::printDisplay()
{
    RCLCPP_INFO_STREAM(node_->get_logger(), COLOR_MGT << LINE << COLOR_RST);
    RCLCPP_INFO_STREAM(node_->get_logger(), COLOR_MGT << "Device Name: " << params.dev_name << COLOR_RST);
    RCLCPP_INFO_STREAM(node_->get_logger(), COLOR_MGT << "Control Unit: " << params.control_unit << COLOR_RST);
    RCLCPP_INFO_STREAM(node_->get_logger(), COLOR_MGT << "Linear Velocity Limit: " << params.angular_limit << COLOR_RST);
    RCLCPP_INFO_STREAM(node_->get_logger(), COLOR_MGT << "Angular Velocity Limit: " << params.linear_limit << COLOR_RST);
    RCLCPP_INFO_STREAM(node_->get_logger(), COLOR_MGT << LINE << COLOR_RST);

}

void ControlUnitNode::controlSelection()
{
    if (params.control_unit == "joy")
    {
        control = std::make_shared<JoystickControl>(node_, params);
    }
    else if (params.control_unit == "keyboard")
    {
        control = std::make_shared<KeyboardControl>(node_, params);
    }
    else if (params.control_unit == "esp8266")
    {
        control = std::make_shared<Esp8266>(node_, params);
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "Invalid control unit specified.");
    }
}

void ControlUnitNode::spin()
{
    rclcpp::spin(node_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    ControlUnitNode control_unit_node;
    control_unit_node.spin();
    rclcpp::shutdown();
    return 0;
}