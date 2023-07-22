#include "command.hpp"

int kfd = 0;

Teleop::Teleop(): Node("Command_node"){
  linear_  = 0.0f;
  angular_ = 0.0f;
  l_scale_ = 2.0f;
  a_scale_ = 0.5f;
  twist_pub_ = this->create_publisher<twistMsg>("cmd_vel",2);
  RCLCPP_INFO(this->get_logger(), "Command Node Started.");
}

void Teleop::keyLoop(){
  char c;
  bool dirty=false;
  twistMsg msg;

  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  while (true){
    if(::read(kfd, &c, 1) < 0){
    RCLCPP_DEBUG(this->get_logger(), "None None");
      perror("read():");
      exit(-1);
    }

    linear_ = 0;
    angular_ = 0;

    switch(c){
      case KEYCODE_L:
        RCLCPP_DEBUG(this->get_logger(), "LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        RCLCPP_DEBUG(this->get_logger(), "RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        RCLCPP_DEBUG(this->get_logger(), "UP");
        linear_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        RCLCPP_DEBUG(this->get_logger(), "DOWN");
        linear_ = -1.0;
        dirty = true;
        break;
      default:
        RCLCPP_DEBUG(this->get_logger(), "None: %x", c);
        dirty = true;
        break;
    }

    RCLCPP_DEBUG(this->get_logger(), "c: %x", c);
    msg.angular.z = a_scale_ * angular_;
    msg.linear.x  = l_scale_ * linear_;

    if(dirty ==true){
      twist_pub_->publish(msg);    
      dirty=false;
    }
  }
}

void quit(int sig){
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  rclcpp::shutdown();
  RCLCPP_INFO(logger, "System Shut Down.");
  exit(0);
}

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  Teleop teleop;
  signal(SIGINT,quit);
  teleop.keyLoop();
  return 0;
}