#include "remote_selector_node.hpp"

RemoteSelector::RemoteSelector(): Node("remote_selector_node"){
    initParam();
}

RemoteSelector::~RemoteSelector(){

}

void RemoteSelector::initParam(){
    this->declare_parameter<std::string>("control_unit", "keyboard");
    control_unit = this->get_parameter("control_unit").as_string();
}