#ifndef __COMMANDER_VISUALIZATION_HPP__
#define __COMMANDER_VISUALIZATION_HPP__

#include "commander_types.hpp"

geometry_msgs::msg::TransformStamped visualizationTf2(State_t state, std::string frame_id = "world");
geometry_msgs::msg::PoseStamped visualizationPath(Position_t pose, std::string frame_id = "world");

#endif