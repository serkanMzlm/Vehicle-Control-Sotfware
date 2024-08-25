#ifndef __CONTOL_UNIT_NODE_HPP__
#define __CONTOL_UNIT_NODE_HPP__

#include "control/esp8266.hpp"
#include "control/joystick.hpp"
#include "control/keyboard.hpp"

class ControlUnitNode
{
private:
    Param_t params;
    std::shared_ptr<ControlUnit> control;
    std::shared_ptr<rclcpp::Node> node_;

public:
    ControlUnitNode();
    ~ControlUnitNode() = default;

    /// @brief Initializes parameters.
    void declareParameters();

    /// @brief Selects the control method based on the control_unit parameter.
    void controlSelection();
    void printDisplay();
    void spin();
};

#endif