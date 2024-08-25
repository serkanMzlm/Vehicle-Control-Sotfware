#include "control/joystick.hpp"

JoystickControl::JoystickControl(std::shared_ptr<rclcpp::Node> node, Param_t params) : ControlUnit(node, params)
{
    if(setup())
    {
        initTopic();
        RCLCPP_INFO(rclcpp::get_logger("joystick_control"), "Joystick control started");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("joystick_control"), "Joystick control failed");
    }
}

bool JoystickControl::setup()
{
    return true;
}

void JoystickControl::initTopic()
{
    joy_sub = _node->create_subscription<joyMsg>("joy", 10, std::bind(&JoystickControl::joyCallback, this, _1));
    joy_pub = _node->create_publisher<joyMsg>("velocity", 10);
}

void JoystickControl::publishVelocity()
{
    joy_pub->publish(joy_data);
}

void JoystickControl::joyCallback(const joyMsg::SharedPtr msg)
{
    joy_data.axes[0] = OFFSET_EXCEPTION(msg->axes[1]);
    joy_data.axes[1] = OFFSET_EXCEPTION(msg->axes[0]);
    joy_data.axes[2] = OFFSET_EXCEPTION(msg->axes[3]);
    joy_data.axes[3] = OFFSET_EXCEPTION(msg->axes[4]);
    joy_data.buttons[0] = joy_data.buttons[0];
    publishVelocity();
}