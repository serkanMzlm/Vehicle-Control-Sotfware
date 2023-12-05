#include "remote_selector_node.hpp"

void RemoteSelector::keyboardCallback(const int32Msg msg){
    bool is_ready = true;
    joy_data.axes[0] = 0.0;
    joy_data.axes[1] = 0.0;
    joy_data.axes[2] = 0.0;
    joy_data.axes[3] = 0.0;
    joy_data.buttons[0] = 1;
    switch(msg.data){
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

    if(is_ready){
        last_msg_timestamp_ = std::chrono::steady_clock::now();
        pub.joy->publish(joy_data);
    }
}

void RemoteSelector::timeOutKeyboard(){
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    elapsed_time = current_time - last_msg_timestamp_;

    if (elapsed_time.count() > 100000000) {
        pub.joy->publish(joy_data);
    } 
}