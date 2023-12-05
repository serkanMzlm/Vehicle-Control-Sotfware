#include "remote_selector_node.hpp"

void RemoteSelector::joyCallback(const joyMsg msg){
    joy_data.axes[0] = OFFSET_EXCEPTION(msg.axes[1]); 
    joy_data.axes[1] = OFFSET_EXCEPTION(msg.axes[0]);
    joy_data.axes[2] = OFFSET_EXCEPTION(msg.axes[3]); 
    joy_data.axes[3] = OFFSET_EXCEPTION(msg.axes[4]);
    buttonChange(msg.buttons[0]);
}

int RemoteSelector::buttonChange(int button_data){
    if(button_data){
        joy_data.buttons[0] = joy_data.buttons[0] ? 0 : 1;
    }
}