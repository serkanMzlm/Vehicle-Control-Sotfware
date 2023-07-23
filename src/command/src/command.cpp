#include "command.hpp"

int kfd = 0;

Teleop::Teleop(): Node("command_node"){
  speed.data[LINEAR] = 0.0f;
  speed.data[ANGULAR] = 0.0f;
  speed.data[L_SCALE] = 1.0f;
  speed.data[A_SCALE] = 0.2f;
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

    speed.data[LINEAR] = 0;
    speed.data[ANGULAR] = 0;

    switch(c){
      case KEYCODE_L:
        RCLCPP_DEBUG(this->get_logger(), "LEFT");
        speed.data[ANGULAR] = 1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        RCLCPP_DEBUG(this->get_logger(), "RIGHT");
        speed.data[ANGULAR] = -1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        RCLCPP_DEBUG(this->get_logger(), "UP");
        speed.data[LINEAR] = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        RCLCPP_DEBUG(this->get_logger(), "DOWN");
        speed.data[LINEAR] = -1.0;
        dirty = true;
        break;
      default:
        RCLCPP_DEBUG(this->get_logger(), "None: %x", c);
        dirty = true;
        break;
    }

    RCLCPP_DEBUG(this->get_logger(), "c: %x", c);
    msg.angular.z = speed.data[A_SCALE] * speed.data[ANGULAR];
    msg.linear.x  = speed.data[L_SCALE] * speed.data[LINEAR];

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