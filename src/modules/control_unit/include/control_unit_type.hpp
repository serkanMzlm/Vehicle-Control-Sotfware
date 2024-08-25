#ifndef __CONTROL_UNIT_TYPE_HPP__
#define __CONTROL_UNIT_TYPE_HPP__

#include <string>
#include <cmath>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32.hpp"

#define COLOR_BLK "\x1b[30m"
#define COLOR_RED "\x1b[31m"
#define COLOR_GRN "\x1b[32m"
#define COLOR_YLW "\x1b[33m"
#define COLOR_BLE "\x1b[34m"
#define COLOR_MGT "\x1b[35m"
#define COLOR_CYN "\x1b[36m"
#define COLOR_WHT "\x1b[37m"
#define COLOR_RST "\x1b[0m"

#define BLACK_STRIPE "\x1b[40m"
#define RED_STRIPE "\x1b[41m"
#define GREEN_STRIPE "\x1b[42m"
#define YELLOW_STRIPE "\x1b[43m"
#define BLUE_STRIPE "\x1b[44m"
#define MAGENTA_STRIPE "\x1b[45m"
#define CYAN_STRIPE "\x1b[46m"
#define WHITE_STRIPE "\x1b[47m"

#define UNDERLINE "\x1b[21m"
#define LINE "============================================"

#define OFFSET 0.1
#define OFFSET_EXCEPTION(X) (abs(X) > OFFSET ? X : 0.0f)

typedef enum
{
    S_HEADER,
    S_LX,
    S_LY,
    S_RX,
    S_RY,
    S_PWR,
    UNDEFINED,
    S_FOOTHER,
    SERIAL_ALL
} Serial_e;

typedef struct
{
    int button[2];
    float joy[2];
    int state = 0;
    uint8_t curr_byte;
    uint8_t prev_byte;
    uint8_t buffer[SERIAL_ALL];
} SerialData_t;

typedef struct
{
    double linear_limit;
    double angular_limit;
    std::string control_unit;
    std::string dev_name;
} Param_t;

using joyMsg = sensor_msgs::msg::Joy;
using int32Msg = std_msgs::msg::Int32;

#endif