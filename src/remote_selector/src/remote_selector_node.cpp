#include "remote_selector_node.hpp"

RemoteSelector::RemoteSelector(): Node("remote_selector_node"){
    initParam();
    memset(rx_data.buffer, 0, sizeof(rx_data.buffer));
    controlSelection();
}

RemoteSelector::~RemoteSelector(){
    close(dev.port);
}

void RemoteSelector::initParam(){
    this->declare_parameter<std::string>("control_unit", "keyboard");
    this->declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
    dev.name = this->get_parameter("device_name").as_string();
    control_unit = this->get_parameter("control_unit").as_string();

    joy_data.axes.resize(4);
    joy_data.buttons.resize(1);
}

void RemoteSelector::controlSelection(){
    if(control_unit == "joy"){
        sub.joy = this->create_subscription<joyMsg>("joy", 10, std::bind(
                &RemoteSelector::joyCallback, this, std::placeholders::_1));
    }else if(control_unit == "keyboard"){
        sub.keyboard = this->create_subscription<int32Msg>("/keypress", 10, std::bind(
            &RemoteSelector::keyboardCallback, this, std::placeholders::_1));
        timer.keyboard = this->create_wall_timer(std::chrono::milliseconds(100), 
                                        std::bind(&RemoteSelector::timeOutKeyboard, this));

    }else if(control_unit == "esp8266"){
         if(initPort()){
            timer.serial = this->create_wall_timer(std::chrono::milliseconds(),
                                    std::bind(&RemoteSelector::serialDataRead, this));                          
        }
    }else{
        std::cout << "control off" << std::endl;
    }
    pub.joy = this->create_publisher<joyMsg>("control_data", 10);
}

double map(double data, double in_min, double in_max, double out_min, double out_max){   
  return ((((data - in_min)*(out_max - out_min))/(in_max - in_min)) + out_min);
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RemoteSelector>());
    rclcpp::shutdown();
    return 0;
}