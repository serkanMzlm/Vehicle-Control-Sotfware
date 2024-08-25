#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>

#include "control/esp8266.hpp"

#define HEADER_ 27
#define FOOTHER_ 71
#define HEADER_LEN 1
#define FOOTHER_LEN 1
#define PAYLOAD_LEN 6
#define BAUDRATE B9600

Esp8266::Esp8266(std::shared_ptr<rclcpp::Node> node, Param_t params) : ControlUnit(node, params)
{
    if (setup())
    {
        initTopic();
        RCLCPP_INFO(rclcpp::get_logger("esp8266_control"), "Esp8266 control started");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("esp8266_control"), "Esp8266 control failed");
    }
}

Esp8266::~Esp8266()
{
    if (port != -1)
    {
        close(port);
    }
}

bool Esp8266::setup()
{
    params.dev_name = "/dev/ttyUSB0";
    memset(rx_data.buffer, 0, sizeof(rx_data.buffer));
    return initPort();
}

bool Esp8266::initPort()
{
    port = open(params.dev_name.c_str(), O_RDWR);
    if (port < 0)
    {
        RCLCPP_ERROR(_node->get_logger(), "Error %d from open: %s", errno, strerror(errno));
        return false;
    }
    if (serialConfig())
    {
        RCLCPP_ERROR(_node->get_logger(), "Configuration settings could not be made.");
        return false;
    }
    usleep(1000000);
    return true;
}

bool Esp8266::serialConfig()
{
    struct termios tty;
    if (tcgetattr(port, &tty) != 0)
    {
        RCLCPP_ERROR(_node->get_logger(), "Error %d from tcgetattr: %s", errno, strerror(errno));
        close(port);
        return false;
    }
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 1;

    cfsetspeed(&tty, BAUDRATE);
    tcflush(port, TCIFLUSH);
    if (tcsetattr(port, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR_STREAM(_node->get_logger(), "Error " << errno << " from tcsetattr : " << strerror(errno));
        close(port);
        return false;
    }
    return true;
}

void Esp8266::initTopic()
{
    timer_ = _node->create_wall_timer(std::chrono::milliseconds(), std::bind(&Esp8266::serialDataCallback, this));
    joy_pub = _node->create_publisher<joyMsg>("velocity", 10);
}

void Esp8266::publishVelocity()
{
    joy_pub->publish(joy_data);
}

void Esp8266::serialDataCallback()
{
    while (read(port, &rx_data.curr_byte, 1) > 0)
    {
        if (rx_data.state == 0)
        {
            if (rx_data.curr_byte == HEADER_ && rx_data.prev_byte == FOOTHER_)
            {
                rx_data.buffer[rx_data.state++] = rx_data.curr_byte;
            }
            else
            {
                rx_data.state = 0;
            }
        }
        else if (rx_data.state < HEADER_LEN + PAYLOAD_LEN)
        {
            rx_data.buffer[rx_data.state++] = rx_data.curr_byte;
        }
        else if (rx_data.state < HEADER_LEN + PAYLOAD_LEN + FOOTHER_LEN)
        {
            rx_data.buffer[rx_data.state++] = rx_data.curr_byte;
            rx_data.state = 0;
            rx_data.prev_byte = rx_data.curr_byte;
            if (rx_data.curr_byte == FOOTHER_)
            {
                joy_data.axes[0] = OFFSET_EXCEPTION(map(static_cast<float>(rx_data.buffer[S_LY]), 0, 200, -1, 1));
                joy_data.axes[1] = OFFSET_EXCEPTION(map(static_cast<float>(rx_data.buffer[S_LX]), 0, 200, 1, -1));
                joy_data.axes[2] = OFFSET_EXCEPTION(map(static_cast<float>(rx_data.buffer[S_RY]), 0, 200, -1, 1));
                joy_data.axes[3] = OFFSET_EXCEPTION(map(static_cast<float>(rx_data.buffer[S_RX]), 0, 200, 1, -1));
                joy_data.buttons[0] = rx_data.buffer[S_PWR];
                publishVelocity();
            }
            else
            {
                rx_data.state = 0;
            }
        }
        else
        {
            rx_data.state = 0;
        }
        rx_data.prev_byte = rx_data.curr_byte;
    }
}