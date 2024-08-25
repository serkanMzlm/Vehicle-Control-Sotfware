#ifndef __ESP8266_CONTROL__
#define __ESP8266_CONTROL__

#include "control_unit.hpp"
#include "math_tools/math_operations.hpp"

class Esp8266 : public ControlUnit
{
private:
    int port;
    SerialData_t rx_data;
    rclcpp::TimerBase::SharedPtr timer_;

private:
    bool setup();   
    void initTopic();
    void publishVelocity();

    /// @brief Initializes the serial port with the specified device name and configuration.
    bool initPort();

    /// @brief Configures the serial port settings.
    bool serialConfig();

public:
    Esp8266(std::shared_ptr<rclcpp::Node> node, Param_t params);
    ~Esp8266();
    void serialDataCallback();
};

#endif