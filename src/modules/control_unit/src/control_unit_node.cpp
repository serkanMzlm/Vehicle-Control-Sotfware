#include "control_unit_node.hpp"

using namespace std::placeholders;

ControlUnit::ControlUnit() : Node("control_unit_node")
{
    initParam();
    memset(rx_data.buffer, 0, sizeof(rx_data.buffer));
    controlSelection();
}

ControlUnit::~ControlUnit()
{
    if (dev.port != -1)
    {
        close(dev.port);
    }
}

void ControlUnit::initParam()
{
    this->declare_parameter<std::string>("control_unit", "keyboard");
    this->declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
    dev.name = this->get_parameter("device_name").as_string();
    control_unit = this->get_parameter("control_unit").as_string();

    joy_data.axes.resize(4);
    joy_data.buttons.resize(1);
}

void ControlUnit::controlSelection()
{
    if (control_unit == "joy")
    {
        sub.joy = this->create_subscription<joyMsg>("joy", 10,
                                                    std::bind(&ControlUnit::joyCallback, this, _1));
    }
    else if (control_unit == "keyboard")
    {
        sub.keyboard = this->create_subscription<int32Msg>("/keypress", 10,
                                                           std::bind(&ControlUnit::keyboardCallback, this, _1));
        timer.keyboard = this->create_wall_timer(std::chrono::milliseconds(100),
                                                 std::bind(&ControlUnit::timeOutKeyboard, this));
    }
    else if (control_unit == "esp8266")
    {
        if (initPort())
        {
            timer.serial = this->create_wall_timer(std::chrono::milliseconds(),
                                                   std::bind(&ControlUnit::serialDataRead, this));
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Invalid control unit specified.");
    }

    pub.joy = this->create_publisher<joyMsg>("velocity", 10);
}

// Serial (ESP8266)
bool ControlUnit::initPort()
{
    dev.port = open(dev.name.c_str(), O_RDWR);
    if (dev.port < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error %d from open: %s", errno, strerror(errno));
        return false;
    }
    if (serialConfig())
    {
        RCLCPP_ERROR(this->get_logger(), "Configuration settings could not be made.");
        return false;
    }
    usleep(1000000);
    return true;
}

bool ControlUnit::serialConfig()
{
    struct termios tty;
    if (tcgetattr(dev.port, &tty) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error %d from tcgetattr: %s", errno, strerror(errno));
        close(dev.port);
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
    tcflush(dev.port, TCIFLUSH);
    if (tcsetattr(dev.port, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Error " << errno << " from tcsetattr : " << strerror(errno));
        close(dev.port);
        return false;
    }
    return true;
}

void ControlUnit::serialDataRead()
{
    while (read(dev.port, &rx_data.curr_byte, 1) > 0)
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
                joy_data.axes[0] = OFFSET_EXCEPTION(mapValue(static_cast<float>(rx_data.buffer[S_LY]), 0, 200, -1, 1));
                joy_data.axes[1] = OFFSET_EXCEPTION(mapValue(static_cast<float>(rx_data.buffer[S_LX]), 0, 200, 1, -1));
                joy_data.axes[2] = OFFSET_EXCEPTION(mapValue(static_cast<float>(rx_data.buffer[S_RY]), 0, 200, -1, 1));
                joy_data.axes[3] = OFFSET_EXCEPTION(mapValue(static_cast<float>(rx_data.buffer[S_RX]), 0, 200, 1, -1));
                joy_data.buttons[0] = rx_data.buffer[S_PWR];
                pub.joy->publish(joy_data);
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

// Keyboard
void ControlUnit::keyboardCallback(const int32Msg msg)
{
    bool is_ready = true;
    joy_data.axes[0] = 0.0;
    joy_data.axes[1] = 0.0;
    joy_data.axes[2] = 0.0;
    joy_data.axes[3] = 0.0;
    joy_data.buttons[0] = 1;

    switch (msg.data)
    {
    case KEYBOARD_W:
        joy_data.axes[0] = 1.0;
        break;
    case KEYBOARD_A:
        joy_data.axes[1] = 0.5;
        break;
    case KEYBOARD_S:
        break;
    case KEYBOARD_D:
        joy_data.axes[1] = -0.5;
        break;
    case KEYBOARD_X:
        joy_data.axes[0] = -1.0;
        break;
    default:
        RCLCPP_DEBUG(this->get_logger(), "None: %x", msg.data);
        is_ready = false;
        break;
    }

    if (is_ready)
    {
        tictoc.tic();
        pub.joy->publish(joy_data);
    }
}

void ControlUnit::timeOutKeyboard()
{
    if (tictoc.toc() > 1.0)
    {
        pub.joy->publish(joy_data);
    }
}

// Joystick
void ControlUnit::joyCallback(const joyMsg msg)
{
    joy_data.axes[0] = OFFSET_EXCEPTION(msg.axes[1]);
    joy_data.axes[1] = OFFSET_EXCEPTION(msg.axes[0]);
    joy_data.axes[2] = OFFSET_EXCEPTION(msg.axes[3]);
    joy_data.axes[3] = OFFSET_EXCEPTION(msg.axes[4]);
    buttonChange(msg.buttons[0]);
}

void ControlUnit::buttonChange(int button_data)
{
    if (button_data)
    {
        joy_data.buttons[0] = joy_data.buttons[0] ? 0 : 1;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlUnit>());
    rclcpp::shutdown();
    return 0;
}