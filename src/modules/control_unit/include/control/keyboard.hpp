#ifndef __KEYBOARD_CONTROL__ 
#define __KEYBOARD_CONTROL__ 

#include "control_unit.hpp"
#include "math_tools/time_utils.hpp"

class KeyboardControl: public ControlUnit
{
private:
    rclcpp::Subscription<int32Msg>::SharedPtr keyboard_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    TicToc tictoc;

private:
    bool setup();
    void initTopic();
    void publishVelocity();

    /// @brief Handles timeout for keyboard input and publishes joystick data if necessary.
    void timeOutKeyboard();

public:
    KeyboardControl(std::shared_ptr<rclcpp::Node> node, Param_t params);
    ~KeyboardControl() = default;
    void keyboardCallback(const int32Msg::SharedPtr msg);
};

#endif