#ifndef __CONTOL_UNIT_NODE_HPP__
#define __CONTOL_UNIT_NODE_HPP__

#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32.hpp"

#include "control_unit_type.hpp"
#include "tic_toc.hpp"
#include "num_tools.hpp"

using joyMsg = sensor_msgs::msg::Joy;
using int32Msg = std_msgs::msg::Int32;

typedef struct
{
    rclcpp::Publisher<joyMsg>::SharedPtr joy;
} Pub_t;

typedef struct
{
    rclcpp::Subscription<joyMsg>::SharedPtr joy;
    rclcpp::Subscription<int32Msg>::SharedPtr keyboard;
} Sub_t;

typedef struct
{
    rclcpp::TimerBase::SharedPtr serial;
    rclcpp::TimerBase::SharedPtr keyboard;
} Timer_t;

class ControlUnit : public rclcpp::Node
{
private:
    Sub_t sub;
    Pub_t pub;
    Timer_t timer;

    Rx_t rx_data;
    Device_t dev;
    joyMsg joy_data;
    std::string control_unit;
    TicToc tictoc;
    
    double linear_velocity_limit;
    double angular_velocity_limit;

public:
    ControlUnit();
    ~ControlUnit();

    /// @brief Initializes parameters.
    void declareParameters();

    /// @brief Selects the control method based on the control_unit parameter.
    void controlSelection();

    /**
     * @brief Initializes the serial port with the specified device name 
     * and configuration.
     *
     * @return true if the port is successfully initialized, false otherwise.
     */
    bool initPort();

    /**
     * @brief Configures the serial port settings.
     *
     * @return true if the configuration is successful, false otherwise.
     */
    bool serialConfig();

    /// @brief Reads data from the serial port and processes it.
    void serialDataRead();

    /**
    * @brief Handles keyboard input by updating joystick axes and publishing data.
    *
    * @param msg The keyboard message to process.
    */
    void keyboardCallback(const int32Msg msg);

    /**
     * @brief Handles timeout for keyboard input and publishes joystick 
     * data if necessary.
     */
    void timeOutKeyboard();

    /**
     * @brief Callback function for processing joystick messages.
     *
     * @param msg The joystick message to process.
     */
    void joyCallback(const joyMsg msg);

    /// @brief Updates the button state based on the input data.
    void buttonChange(int button_data);
};

#endif