#ifndef __COMMANDER_VISUALIZATION_HPP__
#define __COMMANDER_VISUALIZATION_HPP__

#include "commander_types.hpp"

void visualizationTf2(geometry_msgs::msg::TransformStamped& t, State_t state);
bool visualizationPath(poseStampedMsg &pose_stamped, Position_t pose);
void visualizationMarker(markerMsg& marker, float linear_x, float angular_z, int marker_id, State_t state,std::string frame_id = "world");
void visualizationPointCloud(pointCloudMsg &msg, State_t state);

void calculateVector(float linear_x, float angular_z, pointMsg &start, pointMsg &end, int id);

#endif