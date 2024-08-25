#include "control/keyboard.hpp"

#define KEYBOARD_W 87
#define KEYBOARD_A 65
#define KEYBOARD_S 83
#define KEYBOARD_D 68
#define KEYBOARD_X 88

KeyboardControl::KeyboardControl(std::shared_ptr<rclcpp::Node> node, Param_t params) : ControlUnit(node, params)
{
    if(setup())
    {
        initTopic();
        RCLCPP_INFO(rclcpp::get_logger("keyboard_control"), "keyboard control started");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("keyboard_control"), "keyboard control failed");
    }
}

bool KeyboardControl::setup()
{
    return true;
}

void KeyboardControl::initTopic()
{
    keyboard_sub = _node->create_subscription<int32Msg>("/keypress", 10, std::bind(&KeyboardControl::keyboardCallback, this, _1));
    joy_pub = _node->create_publisher<joyMsg>("velocity", 10);

    timer_ = _node->create_wall_timer(std::chrono::milliseconds(100), std::bind(&KeyboardControl::timeOutKeyboard, this));
}

void KeyboardControl::publishVelocity()
{
    joy_pub->publish(joy_data);
}

void KeyboardControl::keyboardCallback(const int32Msg::SharedPtr msg)
{
    bool is_ready = true;
    joy_data.axes[0] = 0.0;
    joy_data.axes[1] = 0.0;
    joy_data.axes[2] = 0.0;
    joy_data.axes[3] = 0.0;
    joy_data.buttons[0] = 1;
    switch (msg->data)
    {
    case KEYBOARD_W:
        RCLCPP_DEBUG(_node->get_logger(), "W");
        joy_data.axes[0] = params.linear_limit;
        is_ready = true;
        break;
    case KEYBOARD_A:
        RCLCPP_DEBUG(_node->get_logger(), "A");
        joy_data.axes[1] = params.angular_limit;
        is_ready = true;
        break;
    case KEYBOARD_S:
        break;
    case KEYBOARD_D:
        RCLCPP_DEBUG(_node->get_logger(), "D");
        joy_data.axes[1] = -params.angular_limit;
        is_ready = true;
        break;
    case KEYBOARD_X:
        RCLCPP_DEBUG(_node->get_logger(), "X");
        is_ready = true;
        joy_data.axes[0] = -params.linear_limit;
        break;
    default:
        RCLCPP_DEBUG(_node->get_logger(), "None: %x", msg->data);
        is_ready = false;
        break;
    }

    if (is_ready)
    {
        tictoc.tic();
        publishVelocity();
    }
}

void KeyboardControl::timeOutKeyboard()
{
    if (tictoc.toc() > 100.0)
    {
        tictoc.tic();
        publishVelocity();
    }
}
