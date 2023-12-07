#include "remote_selector_node.hpp"

bool RemoteSelector::initPort(){
    dev.port = open(dev.name.c_str(), O_RDWR);
    if(dev.port < 0){
        std::cout << "Error " << errno << " from open : " << strerror(errno) << std::endl;
        return false;
    }
    if(serialConfig()){
        std::cout << "Configuration settings could not be made." << std::endl;
        return false;
    }
    usleep(1000000);
    return true;
}

bool RemoteSelector::serialConfig(){
    struct termios tty;
    if(tcgetattr(dev.port, &tty) != 0){
        std::cout << "Error " << errno << " from tcgetattr : " << strerror(errno) << std::endl;
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
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 10;  
    tty.c_cc[VMIN] = 1;

    cfsetspeed(&tty, BAUDRATE);
    tcflush(dev.port, TCIFLUSH);
    if (tcsetattr(dev.port, TCSANOW, &tty) != 0) {
        std::cout << "Error " << errno << " from tcsetattr : " << strerror(errno) << std::endl;
        close(dev.port);
        return false;
    }
    return true;
}

void RemoteSelector::serialDataRead(){
    while(read(dev.port, &rx_data.curr_byte, 1) > 0){
        if(rx_data.state == 0){
            if(rx_data.curr_byte == HEADER_ && rx_data.prev_byte == FOOTHER_){
                rx_data.buffer[rx_data.state++] = rx_data.curr_byte;
            }else{
                rx_data.state = 0;
            }
        }else if(rx_data.state < HEADER_LEN + PAYLOAD_LEN){
            rx_data.buffer[rx_data.state++] = rx_data.curr_byte;
        }else if(rx_data.state < HEADER_LEN + PAYLOAD_LEN + FOOTHER_LEN){
            rx_data.buffer[rx_data.state++] = rx_data.curr_byte;
            rx_data.state = 0;
            rx_data.prev_byte = rx_data.curr_byte;
            if(rx_data.curr_byte == FOOTHER_){
                joy_data.axes[0] = OFFSET_EXCEPTION(map(static_cast<float>(rx_data.buffer[S_LY]), 0, 200, -1, 1));
                joy_data.axes[1] = OFFSET_EXCEPTION(map(static_cast<float>(rx_data.buffer[S_LX]), 0, 200, 1, -1));
                joy_data.axes[2] = OFFSET_EXCEPTION(map(static_cast<float>(rx_data.buffer[S_RY]), 0, 200, -1, 1));
                joy_data.axes[3] = OFFSET_EXCEPTION(map(static_cast<float>(rx_data.buffer[S_RX]), 0, 200, 1, -1));
                joy_data.buttons[0] = rx_data.buffer[S_PWR];
                pub.joy->publish(joy_data);
            }else{
                rx_data.state = 0;
            }
        }else{
            rx_data.state = 0;
        }
        rx_data.prev_byte = rx_data.curr_byte;
    }
}